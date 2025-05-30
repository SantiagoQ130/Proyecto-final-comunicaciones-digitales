/*
 * Arduino R4 WiFi - Modo de prueba optimizado
 * Sistema mejorado de monitoreo de clientes con simulación de sensor
 */

#include <WiFiS3.h>
#include <ArduinoJson.h>

// Configuración WiFi
const char* ssid = "ArduinoR4-Monitor";
const char* password = "12345678";
WiFiServer server(80);

// Variables del sistema
unsigned long tiempoInicio = 0;
unsigned long ultimoReporte = 0;
unsigned long ultimoEscaneo = 0; // Variable que faltaba declarar
const unsigned long intervaloReporte = 3000; // 3 segundos

// Simulación de sensor
float accelX = 0, accelY = 0;
float velocX = 0, velocY = 0;
bool simulacionActiva = true;
float tiempoSim = 0;
unsigned long ultimaSimulacion = 0;

#define TIPO_WIFI 0
#define TIPO_HTTP 1

struct Cliente {
  IPAddress ip;
  unsigned long ultimaConexion;
  unsigned long primeraConexion;
  int peticiones;
  bool activo;
  String userAgent;
  uint8_t tipo; // 0 = WiFi, 1 = HTTP
  String nombreDispositivo; // Podemos intentar identificar el dispositivo
};

Cliente clientes[8];
int numClientes = 0;
unsigned long totalConexiones = 0;
unsigned long ultimaLimpieza = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== Arduino R4 WiFi Monitor ===");
  Serial.println("Iniciando sistema optimizado...");
  
  tiempoInicio = millis();
  ultimaSimulacion = millis();
  ultimoEscaneo = millis(); // Inicializar la variable
  
  // Inicializar clientes
  for(int i = 0; i < 8; i++) {
    clientes[i].activo = false;
    clientes[i].peticiones = 0;
  }
  
  // Configurar AP
  WiFi.config(IPAddress(192, 168, 4, 1));
  
  if (WiFi.beginAP(ssid, password) != WL_AP_LISTENING) {
    Serial.println("ERROR: No se pudo crear el AP");
    while(true) delay(1000);
  }
  
  server.begin();
  
  Serial.println("✓ Sistema iniciado correctamente");
  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  Serial.println("Accede a: http://192.168.4.1");
  Serial.println("================================");
}

void loop() {
  unsigned long ahora = millis();
  
  // Actualizar simulación
  actualizarSimulacion();
  
  // Procesar clientes web
  WiFiClient client = server.available();
  if (client) {
    manejarCliente(client);
  }
  
  // Escanear clientes WiFi cada 5 segundos
  if (ahora - ultimoEscaneo >= 5000) {
    ultimoEscaneo = ahora;
    // Comentamos esta función ya que las funciones WiFi no están disponibles
    // actualizarClientesWiFi();
  }
  
  // Reporte periódico
  if (ahora - ultimoReporte >= intervaloReporte) {
    ultimoReporte = ahora;
    mostrarEstado();
  }
  
  // Limpiar clientes inactivos cada 15 segundos
  if (ahora - ultimaLimpieza >= 15000) {
    ultimaLimpieza = ahora;
    limpiarClientesInactivos();
  }
}

// Función comentada porque WiFi.getNumAssociatedStations() no está disponible en WiFiS3
/*
void actualizarClientesWiFi() {
  // Esta función no funciona con la librería WiFiS3 del Arduino R4
  // Las funciones getNumAssociatedStations() y getStationIP() no están disponibles
  Serial.println("Función actualizarClientesWiFi() no disponible con WiFiS3");
}
*/

void actualizarSimulacion() {
  unsigned long ahora = millis();
  if (ahora - ultimaSimulacion >= 100) { // 10Hz
    ultimaSimulacion = ahora;
    tiempoSim += 0.1;
    
    if (simulacionActiva) {
      // Simulación más realista con patrones variados
      accelX = 1.8 * sin(tiempoSim * 0.4) + 0.5 * cos(tiempoSim * 1.2) + random(-20, 20) / 100.0;
      accelY = 1.2 * cos(tiempoSim * 0.6) + 0.8 * sin(tiempoSim * 0.9) + random(-20, 20) / 100.0;
      
      // Actualizar velocidades con filtrado
      float dt = 0.1;
      if (abs(accelX) > 0.1) {
        velocX += accelX * dt;
      } else {
        velocX *= 0.95; // Decaimiento
        if (abs(velocX) < 0.01) velocX = 0;
      }
      
      if (abs(accelY) > 0.1) {
        velocY += accelY * dt;
      } else {
        velocY *= 0.95;
        if (abs(velocY) < 0.01) velocY = 0;
      }
    }
  }
}

void manejarCliente(WiFiClient client) {
  String peticion = "";
  String linea = "";
  
  // Leer petición
  while (client.connected() && client.available()) {
    char c = client.read();
    if (c == '\n') {
      if (linea.length() == 0) break; // Fin de headers
      if (peticion == "") peticion = linea;
      linea = "";
    } else if (c != '\r') {
      linea += c;
    }
  }
  
  // Registrar cliente
  registrarCliente(client.remoteIP());
  
  // Procesar petición
  if (peticion.indexOf("GET / ") >= 0) {
    enviarPaginaPrincipal(client);
  } else if (peticion.indexOf("GET /data") >= 0) {
    enviarDatosSensor(client);
  } else if (peticion.indexOf("GET /status") >= 0) {
    enviarEstadoSistema(client);
  } else if (peticion.indexOf("GET /toggle") >= 0) {
    alternarSimulacion(client);
  } else {
    enviar404(client);
  }
  
  client.stop();
}

void registrarCliente(IPAddress ip) {
  if (ip == IPAddress(0, 0, 0, 0)) return;
  
  unsigned long ahora = millis();
  
  // Buscar cliente existente
  for(int i = 0; i < numClientes; i++) {
    if(clientes[i].ip == ip && clientes[i].activo) {
      clientes[i].ultimaConexion = ahora;
      clientes[i].peticiones++;
      return;
    }
  }
  
  // Agregar nuevo cliente
  if(numClientes < 8) {
    clientes[numClientes].ip = ip;
    clientes[numClientes].ultimaConexion = ahora;
    clientes[numClientes].primeraConexion = ahora;
    clientes[numClientes].peticiones = 1;
    clientes[numClientes].activo = true;
    clientes[numClientes].tipo = TIPO_HTTP; // Marcar como HTTP ya que llegó por servidor web
    
    // Asignar nombre basado en la última parte de la IP
    switch(ip[3]) {
      case 2: clientes[numClientes].nombreDispositivo = "Teléfono"; break;
      case 3: clientes[numClientes].nombreDispositivo = "Portátil"; break;
      default: clientes[numClientes].nombreDispositivo = "Dispositivo " + String(ip[3]);
    }
    
    numClientes++;
    totalConexiones++;
    
    Serial.print("→ Nuevo cliente: ");
    Serial.print(ip);
    Serial.print(" [Total: ");
    Serial.print(numClientes);
    Serial.println("]");
  }
}

void limpiarClientesInactivos() {
  unsigned long ahora = millis();
  const unsigned long timeout = 30000; // 30 segundos
  
  for(int i = 0; i < numClientes; i++) {
    if(clientes[i].activo && (ahora - clientes[i].ultimaConexion > timeout)) {
      Serial.print("← Cliente desconectado: ");
      Serial.print(clientes[i].ip);
      Serial.print(" [");
      Serial.print(clientes[i].peticiones);
      Serial.println(" peticiones]");
      
      // Mover último elemento
      if(i < numClientes - 1) {
        clientes[i] = clientes[numClientes - 1];
      }
      clientes[numClientes - 1].activo = false;
      numClientes--;
      i--;
    }
  }
}

void mostrarEstado() {
  unsigned long uptime = (millis() - tiempoInicio) / 1000;
  
  Serial.print("Estado: ");
  Serial.print(formatTiempo(uptime));
  Serial.print(" | Clientes: ");
  Serial.print(numClientes);
  Serial.print(" | Total: ");
  Serial.print(totalConexiones);
  Serial.print(" | Sim: ");
  Serial.println(simulacionActiva ? "ON" : "OFF");
  
  if(numClientes > 0) {
    Serial.print("Conectados: ");
    for(int i = 0; i < numClientes; i++) {
      if(clientes[i].activo) {
        Serial.print(clientes[i].ip);
        Serial.print("(");
        Serial.print(clientes[i].peticiones);
        Serial.print(clientes[i].tipo == TIPO_WIFI ? "W" : "H"); // W=WiFi, H=HTTP
        Serial.print(") ");
      }
    }
    Serial.println();  
  }
}

String formatTiempo(unsigned long seg) {
  unsigned long h = seg / 3600;
  unsigned long m = (seg % 3600) / 60;
  unsigned long s = seg % 60;
  
  String resultado = "";
  if(h > 0) resultado += String(h) + "h";
  if(m > 0) resultado += String(m) + "m";
  resultado += String(s) + "s";
  return resultado;
}

void enviarPaginaPrincipal(WiFiClient client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println();
  
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Arduino R4 Monitor</title>";
  html += "<style>";
  html += "body{font-family:Arial;margin:0;background:#f5f5f5;color:#333}";
  html += ".container{max-width:800px;margin:0 auto;padding:20px}";
  html += ".card{background:white;border-radius:8px;padding:20px;margin:15px 0;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
  html += ".header{background:#2c3e50;color:white;text-align:center;padding:20px;border-radius:8px}";
  html += ".btn{background:#3498db;color:white;border:none;padding:10px 20px;border-radius:5px;cursor:pointer;margin:5px}";
  html += ".btn:hover{background:#2980b9}";
  html += ".status{display:inline-block;padding:4px 8px;border-radius:4px;font-size:12px;font-weight:bold}";
  html += ".active{background:#27ae60;color:white}.inactive{background:#e74c3c;color:white}";
  html += ".grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:10px 0}";
  html += ".value{font-size:18px;font-weight:bold;color:#2c3e50}";
  html += "table{width:100%;border-collapse:collapse;margin-top:10px}";
  html += "table td, table th{padding:8px;text-align:left}";
  html += "table tr:nth-child(even){background-color:#f2f2f2}";
  html += "</style></head><body>";
  
  html += "<div class='container'>";
  html += "<div class='header'><h1>🔧 Arduino R4 Monitor</h1><p>Sistema de monitoreo en tiempo real</p></div>";
  
  html += "<div class='card'>";
  html += "<h2>📊 Datos del Sensor (Simulados)</h2>";
  html += "<div id='sensorData'>Cargando...</div>";
  html += "<button class='btn' onclick='toggleSim()'>Toggle Simulación</button>";
  html += "</div>";
  
  html += "<div class='card'>";
  html += "<h2>🌐 Estado del Sistema</h2>";
  html += "<div id='systemStatus'>Cargando...</div>";
  html += "<button class='btn' onclick='updateAll()'>Actualizar Todo</button>";
  html += "</div>";
  
  html += "</div>";
  
  html += "<script>";
  html += "function updateSensor(){";
  html += "fetch('/data').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('sensorData').innerHTML=";
  html += "'<div class=\"grid\">'+";
  html += "'<div>Accel X: <span class=\"value\">'+d.accelX.toFixed(3)+'</span> m/s²</div>'+";
  html += "'<div>Accel Y: <span class=\"value\">'+d.accelY.toFixed(3)+'</span> m/s²</div>'+";
  html += "'<div>Veloc X: <span class=\"value\">'+d.velocX.toFixed(3)+'</span> m/s</div>'+";
  html += "'<div>Veloc Y: <span class=\"value\">'+d.velocY.toFixed(3)+'</span> m/s</div>'+";
  html += "'</div><p>Estado: <span class=\"status '+(d.simulation?'active':'inactive')+'\">'+";
  html += "(d.simulation?'ACTIVA':'PAUSADA')+'</span></p>'}).catch(e=>console.log(e))}";
  
  html += "function updateStatus(){";
  html += "fetch('/status').then(r=>r.json()).then(d=>{";
  html += "let html='<div class=\"grid\">';";
  html += "html+='<div>Tiempo Activo: <span class=\"value\">'+d.uptime+'</span></div>';";
  html += "html+='<div>Clientes: <span class=\"value\">'+d.clients+'</span></div>';";
  html += "html+='<div>Total Conexiones: <span class=\"value\">'+d.total+'</span></div>';";
  html += "html+='<div>IP: <span class=\"value\">'+d.ip+'</span></div>';";
  html += "html+='</div>';";

  html += "if(d.activeClients.length>0){";
  html += "html+='<h3>Dispositivos Conectados:</h3>';";
  html += "html+='<table style=\"width:100%;border-collapse:collapse;margin-top:10px;\">';";
  html += "html+='<tr style=\"background:#2c3e50;color:white;\"><th>Dispositivo</th><th>IP</th><th>Tipo</th><th>Peticiones</th><th>Tiempo</th></tr>';";

  html += "d.activeClients.forEach(c=>{";
  html += "html+='<tr style=\"border-bottom:1px solid #ddd;\">';";
  html += "html+='<td>'+ (c.device || 'Desconocido') +'</td>';";
  html += "html+='<td>'+c.ip+'</td>';";
  html += "html+='<td><span class=\"status '+(c.type=='http'?'active':'inactive')+'\">'+c.type.toUpperCase()+'</span></td>';";
  html += "html+='<td>'+c.requests+'</td>';";
  html += "html+='<td>'+c.connected+'</td>';";
  html += "html+='</tr>';});";

  html += "html+='</table>';";
  html += "}else{html+='<p><em>No hay dispositivos conectados</em></p>';}";
  html += "document.getElementById('systemStatus').innerHTML=html}).catch(e=>console.log(e))}";
  
  html += "function toggleSim(){fetch('/toggle').then(r=>r.json()).then(d=>updateSensor())}";
  html += "function updateAll(){updateSensor();updateStatus()}";
  html += "updateAll();setInterval(updateSensor,1000);setInterval(updateStatus,3000);";
  html += "</script></body></html>";
  
  client.print(html);
}

void enviarDatosSensor(WiFiClient client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:application/json");
  client.println();
  
  StaticJsonDocument<200> doc;
  doc["accelX"] = accelX;
  doc["accelY"] = accelY;
  doc["velocX"] = velocX;
  doc["velocY"] = velocY;
  doc["simulation"] = simulacionActiva;
  doc["timestamp"] = millis();
  
  String json;
  serializeJson(doc, json);
  client.print(json);
}

void enviarEstadoSistema(WiFiClient client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:application/json");
  client.println();
  
  unsigned long uptime = (millis() - tiempoInicio) / 1000;
  
  StaticJsonDocument<1024> doc;
  doc["uptime"] = formatTiempo(uptime);
  doc["clients"] = numClientes;
  doc["total"] = totalConexiones;
  doc["ip"] = WiFi.localIP().toString();
  doc["ssid"] = ssid;
  
  JsonArray clientsArray = doc.createNestedArray("activeClients");
  for(int i = 0; i < numClientes; i++) {
    if(clientes[i].activo) {
      JsonObject clientObj = clientsArray.createNestedObject();
      clientObj["ip"] = clientes[i].ip.toString();
      clientObj["requests"] = clientes[i].peticiones;
      
      unsigned long conectado = (millis() - clientes[i].primeraConexion) / 1000;
      clientObj["connected"] = formatTiempo(conectado);
      clientObj["type"] = clientes[i].tipo == TIPO_WIFI ? "wifi" : "http";
      
      if(clientes[i].ip[3] == 2) {
        clientObj["device"] = "Teléfono Principal";
      } else if(clientes[i].ip[3] == 3) {
        clientObj["device"] = "Portátil";
      } else {
        clientObj["device"] = "Dispositivo " + String(clientes[i].ip[3]);
      }
    }
  }
  
  String json;
  serializeJson(doc, json);
  client.print(json);
}

void alternarSimulacion(WiFiClient client) {
  simulacionActiva = !simulacionActiva;
  
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:application/json");
  client.println();
  
  StaticJsonDocument<100> doc;
  doc["active"] = simulacionActiva;
  doc["message"] = simulacionActiva ? "Activada" : "Pausada";
  
  String json;
  serializeJson(doc, json);
  client.print(json);
  
  Serial.print("Simulación ");
  Serial.println(simulacionActiva ? "ACTIVADA" : "PAUSADA");
}

void enviar404(WiFiClient client) {
  client.println("HTTP/1.1 404 Not Found");
  client.println("Content-type:text/plain");
  client.println();
  client.print("Página no encontrada");
}
