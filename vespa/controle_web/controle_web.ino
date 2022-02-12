/*******************************************************************************
* Controle web de um robo com a Vespa por WebSocket
* (v1.0 - 25/10/2021)
*
* Copyright 2021 RoboCore.
* Interface web escrita por Lenz (25/10/2021).
* Programa do ESP escrito por Francois (25/10/2021).
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version (<https://www.gnu.org/licenses/>).
*******************************************************************************/

// --------------------------------------------------
// Bibliotecas

#include <WiFi.h>
#include <AsyncTCP.h> // https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h> // https://github.com/me-no-dev/ESPAsyncWebServer
#include <ArduinoJson.h> // https://arduinojson.org/

#include <RoboCore_Vespa.h>
#include <Ultrasonic.h>  // https://github.com/RoboCore/Ultrasonic
#include "client_web.h"  // Pagina web principal

// --------------------------------------------------
// Variaveis

// web server assincrono na porta 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// LED
const uint8_t PIN_LED = 15;

// JSON aliases
const char *ALIAS_ANGULO = "angulo";
const char *ALIAS_VELOCIDADE = "velocidade";
const char *ALIAS_VBAT = "vbat";

// variaveis da Vespa
VespaMotors motores;
VespaBattery vbat;
const uint32_t TEMPO_ATUALIZACAO_VBAT = 5000; // [ms]
uint32_t timeout_vbat;

// --------------------------------------------------
// Prototipos

void configurar_servidor_web(void);
void handleWebSocketMessage(void *, uint8_t *, size_t);
void onEvent(AsyncWebSocket *, AsyncWebSocketClient *, AwsEventType,
             void *, uint8_t *, size_t);

// --------------------------------------------------
// Sensor de distância - proteção conta colisão frontal
HC_SR04 sensor1(33, 32); // (trigger, echo)
#define distancia_limite sensor1.distance() > 30
// --------------------------------------------------
// --------------------------------------------------

void setup(){
  // configura a comunicacao serial
  Serial.begin(115200);
  Serial.println("RoboCore - Vespa Joystick");
  Serial.println("\t(v1.0 - 25/10/21)\n");

  // configura o LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // configura o ponto de acesso (Access Point)
  Serial.print("Configurando a rede Wi-Fi... ");
  const char *mac = WiFi.macAddress().c_str(); // obtem o MAC
  char ssid[] = "Vespa-xxxxx"; // mascara do SSID (ate 63 caracteres)
  char *senha = "rc-vespa"; // senha padrao da rede (no minimo 8 caracteres)
  // atualiza o SSID em funcao do MAC
  for(uint8_t i=6 ; i < 11 ; i++){
    ssid[i] = mac[i+6];
  }
  if(!WiFi.softAP(ssid, senha)){
    Serial.println("ERRO");
    // trava a execucao
    while(1){
      digitalWrite(PIN_LED, HIGH);
      delay(100);
      digitalWrite(PIN_LED, LOW);
      delay(100);
    }
  }
  Serial.println("OK");
  Serial.printf("A rede \"%s\" foi gerada\n", ssid);
  Serial.print("IP de acesso: ");
  Serial.println(WiFi.softAPIP());

  // configura e iniciar o servidor web
  configurar_servidor_web();
  server.begin();
  Serial.println("Servidor iniciado\n");
}

// --------------------------------------------------

void loop() {
  // le a tensao da bateria e envia para o cliente
  if(millis() > timeout_vbat){
    // atualiza se houver clientes conectados
    if(ws.count() > 0){
      // le a tensao da bateria
      uint32_t tensao = vbat.readVoltage();
      
      // cria a mensagem
      const int json_tamanho = JSON_OBJECT_SIZE(1); // objeto JSON com um membro
      StaticJsonDocument<json_tamanho> json;
      json[ALIAS_VBAT] = tensao;
      size_t mensagem_comprimento = measureJson(json);
      char mensagem[mensagem_comprimento + 1];
      serializeJson(json, mensagem, (mensagem_comprimento+1));
      mensagem[mensagem_comprimento] = 0; // EOS (mostly for debugging)
  
      // send the message
      ws.textAll(mensagem, mensagem_comprimento);
      Serial.printf("Tensao atualizada: %u mV\n", tensao);
    }
    
    timeout_vbat = millis() + TEMPO_ATUALIZACAO_VBAT; // atualiza
  }
}

// --------------------------------------------------
// --------------------------------------------------

// Configurar o servidor web
void configurar_servidor_web(void) {
  ws.onEvent(onEvent); // define o manipulador do evento do WebSocket
  server.addHandler(&ws); // define o manipulador do WebSocket no servidor
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ // define a resposta da pagina padrao
    request->send_P(200, "text/html", index_html);
  });
}

// --------------------------------------------------

// Manipulador para mensagens WebSocket
//  @param (arg) : xxx [void *]
//         (data) : xxx [uint8_t *]
//         (length) : xxx [size_t]
void handleWebSocketMessage(void *arg, uint8_t *data, size_t length) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == length && info->opcode == WS_TEXT) {
    data[length] = 0;
//    Serial.printf("Incoming WS data: \"%s\"\n", (char*)data); // debug

    // verifica se eh para controlar os motores
    if(strstr(reinterpret_cast<char*>(data), ALIAS_VELOCIDADE) != nullptr){
      // cria um documento JSON
      const int json_tamanho = JSON_OBJECT_SIZE(2); // objeto JSON com dois membros
      StaticJsonDocument<json_tamanho> json;
      DeserializationError erro = deserializeJson(json, data, length);
      
      // extrai os valores do JSON
      int16_t angulo = json[ALIAS_ANGULO]; // [0;360]
      int16_t velocidade = json[ALIAS_VELOCIDADE]; // [0;100]

      // debug
      Serial.print("Velocidade: ");
      Serial.print(velocidade);
      Serial.print(" | Angulo: ");
      Serial.println(angulo);

      // atualiza os motores
      if((angulo > 80) && (angulo < 100)){
        if (distancia_limite)
          motores.forward(velocidade);
      } else if((angulo > 260) && (angulo < 280)){
        motores.backward(velocidade);
      } else if((angulo > 170) && (angulo < 190)){
        motores.turn(0, velocidade);
      } else if((angulo > 350) || (angulo < 10)){
        motores.turn(velocidade, 0);
      } else if((angulo > 100) && (angulo < 170)){
        if (distancia_limite)
          motores.turn(velocidade * 7 / 10, velocidade);
      } else if((angulo > 10) && (angulo < 80)){
        if (distancia_limite)
          motores.turn(velocidade, velocidade * 7 / 10);
      } else if((angulo > 190) && (angulo < 260)){
        motores.turn(-1 * velocidade * 7 / 10, -1 * velocidade);
      } else if((angulo > 280) && (angulo < 350)){
        motores.turn(-1 * velocidade, -1 * velocidade * 7 / 10);
      } else {
        motores.stop();
      }
    } else {
      Serial.printf("Recebidos dados invalidos (%s)\n", data);
    }
  }
}

// --------------------------------------------------

// Manipulador dos eventos do WebSocket
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t length) {
  switch (type) {
    case WS_EVT_CONNECT: {
      digitalWrite(PIN_LED, HIGH); // acende o LED
      // permitir apenas um cliente conectado
      if(ws.count() == 1){ // o primeiro cliente ja eh considerado como conectado
        Serial.printf("Cliente WebSocket #%u conectado de %s\n", client->id(), client->remoteIP().toString().c_str());
      } else {
        Serial.printf("Cliente WebSocket #%u de %s foi rejeitado\n", client->id(), client->remoteIP().toString().c_str());
        ws.close(client->id());
      }
      break;
    }
    case WS_EVT_DISCONNECT: {
      if(ws.count() == 0){
        digitalWrite(PIN_LED, LOW); // apaga o LED
      }
      Serial.printf("Cliente WebSocket #%u desconectado\n", client->id());
      break;
    }
    case WS_EVT_DATA: {
      handleWebSocketMessage(arg, data, length);
      break;
    }
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// --------------------------------------------------
