#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <Edubot.h>
#include <SPI.h>
#include <LCD_Mega.h>
#define mainDELAY_LOOP_COUNT 300000

Edubot robot;
LCD_Mega lcd;

// Criação dos prótotipos
void Rodas(void *pvParameters );
void Infravermelhos (void *pvParameters);
void Cerebro (void *pvParameters);
void LCD (void *pvParameters);

// Criação dos Handles das tasks, das queues e dos semáforos
TaskHandle_t xInfraHandle;
TaskHandle_t xLCD;
QueueHandle_t xQueueInfra;
QueueHandle_t xQueueRodas;
QueueHandle_t xQueueLCD;
SemaphoreHandle_t xSemaforoInfra;
SemaphoreHandle_t xSemaforoRodas;

const TickType_t xWait = 10 / portTICK_PERIOD_MS;

void setup() {
  Serial1.begin(9600);
  robot.Setup();
  lcd.begin();
  lcd.LcdClear();
  delay(100);

  // Criação das queues e dos semáforos
  xQueueInfra = xQueueCreate(1, sizeof(int));
  xQueueRodas = xQueueCreate(1, sizeof(int));
  xQueueLCD = xQueueCreate(1, sizeof(int));
  vSemaphoreCreateBinary( xSemaforoInfra );
  vSemaphoreCreateBinary( xSemaforoRodas );

  // Criação das Tasks com as respetivas prioridades caso a criação das queues e dos semáforos
  // forem bem sucedidas
  if (xQueueLCD != NULL && xQueueInfra != NULL && xQueueRodas != NULL && xSemaforoInfra != NULL && xSemaforoRodas != NULL) {
    xTaskCreate(Cerebro, "Cerebro", 200, NULL, 2, NULL);
    xTaskCreate(Rodas, "Rodas", 200, NULL, 1, NULL);
    xTaskCreate(Infravermelhos, "Infra", 200, NULL, 1, &xInfraHandle);
    xTaskCreate(LCD, "LCD", 200, NULL, 3, &xLCD);

    // Inicia o escalonador
    vTaskStartScheduler();
  } else {
  }
  for (;;);
}

/*********************************RODAS********************************/
void Rodas (void* pvParameters) {
  short int command;
  volatile unsigned long ul;
  portBASE_TYPE xStateQueueRodas;

  for (;;) {
    xSemaphoreTake(xSemaforoRodas, portMAX_DELAY);
    xStateQueueRodas = xQueueReceive(xQueueRodas, &command, portMAX_DELAY);

    switch (command) {
      case 1: // Andar em frente
        robot.setMotorSpeed(robot.DIR, 0.9 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, 0.9 * robot.baseSpeed);
        break;

      case 2: // Rodar um pouco no sentido contrário ao dos ponteiros do relógio
        robot.setMotorSpeed(robot.DIR, 0.9 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, 0 * robot.baseSpeed);
        break;

      case 3: // Rodar um pouco no sentido dos ponteiros do relógio
        robot.setMotorSpeed(robot.DIR, 0 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, 0.9 * robot.baseSpeed);
        break;

      case 4: // Rodar muito no sentido contrário ao dos ponteiros do relógio
        robot.setMotorSpeed(robot.DIR, 1.7 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, 0 * robot.baseSpeed);
        break;

      case 5: // Rodar muito no sentido dos ponteiros do relógio
        robot.setMotorSpeed(robot.DIR, 0 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, 1.7 * robot.baseSpeed);
        break;

      case 6:
        robot.setMotorSpeed(robot.DIR, 1.4 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, 0 * robot.baseSpeed);
        break;

      case 7:
        robot.setMotorSpeed(robot.DIR, 0 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, 1.4 * robot.baseSpeed);
        break;

      case 8:
        robot.setMotorSpeed(robot.DIR, -0.9 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, 0.9 * robot.baseSpeed);
        break;

      case 9:
        robot.setMotorSpeed(robot.DIR, 1.4 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, -0.5 * robot.baseSpeed);
        break;

      case 11:
        robot.setMotorSpeed(robot.DIR, -0.5 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, 1.4 * robot.baseSpeed);
        break;

      default: // Para o robô
        robot.setMotorSpeed(robot.DIR, 0 * robot.baseSpeed);
        robot.setMotorSpeed(robot.ESQ, 0 * robot.baseSpeed);
        break;
    }
  }
}

/**************************************INFRAVERMELHOS************************************/
void Infravermelhos(void* pvParameters) {
  portBASE_TYPE xQueueState;
  short int var_infra;
  for (;;) {

    // Espera pelo semáforo dado pelo cérebro
    xSemaphoreTake(xSemaforoInfra, portMAX_DELAY);
    robot.readSensors();

    // Escolhe o comportamento que o robo deve tomar consoante os valores recebidos pelos
    // sensores infravermelhos
    if ((robot.sensor_values[3] < robot.sensor_thresh) && (robot.sensor_values[2] > robot.sensor_thresh) &&
        (robot.sensor_values[1] > robot.sensor_thresh) && (robot.sensor_values[0] < robot.sensor_thresh)) {

      var_infra = 1; // Andar em frente

    } else if ((robot.sensor_values[3] < robot.sensor_thresh) && (robot.sensor_values[2] > robot.sensor_thresh) &&
               (robot.sensor_values[1] < robot.sensor_thresh) && (robot.sensor_values[0] < robot.sensor_thresh)) {

      var_infra = 2; // Rodar um pouco no sentido contrário ao dos ponteiros do relógio

    } else if ((robot.sensor_values[3] < robot.sensor_thresh) && (robot.sensor_values[2] < robot.sensor_thresh) &&
               (robot.sensor_values[1] > robot.sensor_thresh) && (robot.sensor_values[0] < robot.sensor_thresh)) {

      var_infra = 3; // Rodar um pouco no sentido dos ponteiros do relógio

    } else if ((robot.sensor_values[3] > robot.sensor_thresh) && (robot.sensor_values[2] < robot.sensor_thresh) &&
               (robot.sensor_values[1] < robot.sensor_thresh) && (robot.sensor_values[0] < robot.sensor_thresh)) {

      var_infra = 4; // Rodar muito no sentido contrário ao dos ponteiros do relógio

    } else if ((robot.sensor_values[3] < robot.sensor_thresh) && (robot.sensor_values[2] < robot.sensor_thresh) &&
               (robot.sensor_values[1] < robot.sensor_thresh) && (robot.sensor_values[0] > robot.sensor_thresh)) {

      var_infra = 5; // Rodar muito no sentido dos ponteiros do relógio

    } else if ((robot.sensor_values[3] > robot.sensor_thresh) && (robot.sensor_values[2] > robot.sensor_thresh) &&
               (robot.sensor_values[1] < robot.sensor_thresh) && (robot.sensor_values[0] < robot.sensor_thresh)) {

      var_infra = 6;

    } else if ((robot.sensor_values[3] < robot.sensor_thresh) && (robot.sensor_values[2] < robot.sensor_thresh) &&
               (robot.sensor_values[1] > robot.sensor_thresh) && (robot.sensor_values[0] > robot.sensor_thresh)) {

      var_infra = 7;

    } else if ((robot.sensor_values[3] < robot.sensor_thresh) && (robot.sensor_values[2] < robot.sensor_thresh) &&
               (robot.sensor_values[1] < robot.sensor_thresh) && (robot.sensor_values[0] < robot.sensor_thresh)) {

      var_infra = 8; // Todos os sensores estão fora da linha

    } else if ((robot.sensor_values[3] > robot.sensor_thresh) && (robot.sensor_values[2] > robot.sensor_thresh) &&
               (robot.sensor_values[1] > robot.sensor_thresh) && (robot.sensor_values[0] > robot.sensor_thresh)) {

      var_infra = 9;

    } else if ((robot.sensor_values[3] > robot.sensor_thresh) && (robot.sensor_values[2] > robot.sensor_thresh) &&
               (robot.sensor_values[1] > robot.sensor_thresh) && (robot.sensor_values[0] < robot.sensor_thresh)) {

      var_infra = 10;

    } else if ((robot.sensor_values[3] < robot.sensor_thresh) && (robot.sensor_values[2] > robot.sensor_thresh) &&
               (robot.sensor_values[1] > robot.sensor_thresh) && (robot.sensor_values[0] > robot.sensor_thresh)) {

      var_infra = 11;

    }

    // Envia para a queue o commando a realizar
    xQueueState = xQueueSendToBack(xQueueInfra, &var_infra, 0);
  }
}


/**************************************CÉREBRO*************************************/
void Cerebro(void* pvParameters) {
  short int var_infra, aux = 0, cont_preto = 0, cont_vel = 0, cont_ignorar = 0, tipo_intersecao = 0;
  short int indice = -1, var_mov[20] = {0}, var_inter[20] = {0}, ignorar_inter = 0, num_curvas = 0;
  boolean ignorar = false, retornar = false, curva = false;
  portBASE_TYPE xStateQueueInfra;
  portBASE_TYPE xQueueState;
  portBASE_TYPE xQueueState_LCD;

  for (;;) {
    if (cont_ignorar == 0) {
      // Faz um give de um semáforo para que a Task dos Infravermelhos possa correr
      xSemaphoreGive( xSemaforoInfra );
      // Espera que a Task dos infravermelhos envie o seu valor na queue
      xStateQueueInfra = xQueueReceive(xQueueInfra, &var_infra, portMAX_DELAY);
    }


    /*************************DETEÇÃO DO TIPO DE INTERSEÇÃO****************************/
    if (var_infra == 9 && cont_vel == 0) {
      cont_preto++;
      if (cont_preto > 10) {
        aux = 1;
      }
    }

    if (var_infra == 10 && cont_vel == 0) {
      cont_preto++;
      if (cont_preto > 10) {
        aux = 2;
      }
    }

    if (var_infra == 11 && cont_vel == 0) {
      cont_preto++;
      if (cont_preto > 10) {
        aux = 3;
      }
    }
    /**********************************************************************************/


    /******************MOVIMENTAÇÃO DE ACORDO COM O TIPO DE INTERSEÇÃO*****************/
    if (aux == 1) {
      if (var_infra == 9 && cont_vel < 2) {
        cont_preto++;
        cont_vel = 1;
        if (cont_preto >= 60) {
          cont_vel = 2;
        }
      } else if (cont_vel == 1) {
        cont_vel = 2;
      }

      if (cont_vel > 1) {
        cont_vel++;
        if (cont_vel > 35) {
          if (cont_preto > 15 && cont_preto < 60) {
            if (var_infra == 8 && cont_ignorar == 0) {
              tipo_intersecao = 2;
              if (ignorar_inter == 0) {
                indice++;
                var_inter[indice] = 2;
                var_mov[indice] = 1;
              }
            } else if (var_infra < 4 && cont_ignorar == 0) {
              tipo_intersecao = 1;
              if (ignorar_inter == 0) {
                indice++;
                var_inter[indice] = 1;
                var_mov[indice] = 1;
              }
            } else {
            }

            if (retornar == true && var_mov[indice] == 1) {
              var_infra = 11;
            }

            if (retornar == true && var_mov[indice] == 2) {
              var_infra = 1;
              cont_ignorar = 40;
            }

            if ((retornar == false) || (retornar == true && var_mov[indice] == 3)) {
              var_infra = 9;
            }

            ignorar = true;
          } else if (cont_preto >= 60) {
            if (cont_ignorar == 0) {
              var_infra = 8;
              ignorar_inter = 1;
              retornar = true;
            }
            cont_ignorar++;
            if (cont_ignorar > 140) {
              cont_preto = 0;
              cont_vel = 0;
              cont_ignorar = 0;
              aux = 0;
              tipo_intersecao = 0;
              for (int i = indice; i > -1; i--) {
                if (var_mov[i] != 0) {
                  if (var_mov[i] == 1) {
                    Serial1.print("D");
                  } else if (var_mov[i] == 2) {
                    Serial1.print("F");
                  } else {
                    Serial1.print("E");
                  }
                }
              }
            }
          }
        }
      }
    }

    if (aux == 2) {
      cont_vel++;
      if (cont_vel > 65) {
        if (var_infra == 8 && cont_ignorar == 0) {
          curva = true;
          var_infra = 9;
        } else if (cont_ignorar == 0) {
          tipo_intersecao = 3;
          if (ignorar_inter == 0) {
            indice++;
            var_inter[indice] = 3;
            var_mov[indice] = 1;
          }
          if (retornar == true && var_mov[indice] == 2) {
            var_infra = 1;
            cont_ignorar = 40;
          }
          if ((retornar == false) || (retornar == true && var_mov[indice] == 3)) {
            var_infra = 9;
          }
        }
        ignorar = true;
      }
    }

    if (aux == 3) {
      cont_vel++;
      if (cont_vel > 55) {
        if (var_infra == 8 && cont_ignorar == 0) {
          var_infra = 11;
          curva = true;
        } else if (cont_ignorar == 0) {
          if (retornar == true && var_mov[indice] == 1) {
            var_infra = 11;
          }

          if ((retornar == false) || (retornar == true && var_mov[indice] == 2)) {
            var_infra = 1;
            if (retornar == true && var_mov[indice] == 2) {
              cont_ignorar = 40;
            }
          }
          tipo_intersecao = 4;
          if (ignorar_inter == 0) {
            indice++;
            var_inter[indice] = 4;
            var_mov[indice] = 2;
          }
        }
        ignorar = true;
      }
    }
    /************************************************************************************/


    /**************TODOS OS SENSORES FORA DA LINHA PRETA*************/
    if (var_infra == 8 && aux == 0) {
      cont_preto++;
      if (cont_preto >= 190 && retornar == false) {
        if (cont_ignorar == 0) {

          /* cruzamento */
          if (var_mov[indice] == 3 && var_inter[indice] == 1) {
            var_mov[indice] = 0;
            var_inter[indice] = 0;
            indice--;
            ignorar_inter++;
          }

          /* entrocamento_t */
          if (var_mov[indice] == 3 && var_inter[indice] == 2) {
            var_mov[indice] = 0;
            var_inter[indice] = 0;
            indice--;
            ignorar_inter++;
          }

          /* entrocamento_t90_E */
          if (var_mov[indice] == 2 && var_inter[indice] == 3) {
            var_mov[indice] = 0;
            var_inter[indice] = 0;
            indice--;
            ignorar_inter++;
          }

          /* entrocamento_t90_D */
          if (var_mov[indice] == 3 && var_inter[indice] == 4) {
            var_mov[indice] = 0;
            var_inter[indice] = 0;
            indice--;
            ignorar_inter++;
          }
        }
      }

      if (cont_preto > 200 && retornar == false) {
        if (cont_ignorar == 0) {
          tipo_intersecao = 5;
          var_infra = 8;

          if (var_mov[indice] < 3 && var_inter[indice] == 1) {
            var_mov[indice] = var_mov[indice] + 1;
            ignorar_inter++;
          }

          if (var_mov[indice] < 3 && var_inter[indice] == 2) {
            var_mov[indice] = var_mov[indice] + 2;
            ignorar_inter++;
          }

          if (var_mov[indice] < 2 && var_inter[indice] == 3) {
            var_mov[indice] = var_mov[indice] + 1;
            ignorar_inter++;
          }

          if (var_mov[indice] < 3 && var_inter[indice] == 4) {
            var_mov[indice] = var_mov[indice] + 1;
            ignorar_inter++;
          }
        }
        cont_ignorar++;
        if (cont_ignorar > 135) {
          cont_preto = 0;
          cont_ignorar = 0;
          tipo_intersecao = 0;
        }
      }
    } else if (aux == 0 && var_infra < 7) {
      cont_preto = 0;
    }
    /*****************************************************************/


    /*****************CÓDIGO PARA IGNORAR******************/
    if (ignorar == true) {
      if (cont_ignorar == 0 || cont_ignorar == 40) {
        aux = 0;
      }
      cont_ignorar++;
      if (cont_ignorar >= 55) {
        cont_vel = 0;
        cont_ignorar = 0;
        cont_preto = 0;
        tipo_intersecao = 0;
        ignorar = false;

        if (retornar == true) {
          if (curva == false) {
            indice--;
          } else {
            curva = false;
            if (indice == -1) {
              num_curvas--;
            }
          }
        } else {
          if (curva == true) {
            curva = false;
            if (indice == -1) {
              num_curvas++;
            }
          } else {
            ignorar_inter--;
          }
          if (ignorar_inter < 0) {
            ignorar_inter = 0;
          }
        }
      }
    }
    /******************************************************/

    if (cont_preto != 0 && cont_ignorar == 0) {
      var_infra = 1;
    }

    if (retornar == true) {
      tipo_intersecao = 6;
    }

    if (retornar == true && indice == -1 && num_curvas == 0) {
      var_infra = 12;
      var_mov[20] = {0};
      var_inter[20] = {0};
      ignorar_inter = 0;
      retornar = false;
      tipo_intersecao = 7;
    }

    xQueueState_LCD = xQueueSendToBack(xQueueLCD, &tipo_intersecao, 0);

    xQueueState = xQueueSendToBack(xQueueRodas, &var_infra, 0);
    // É aumentada a prioridade da Task LDC para poder fazer um peek do valor enviado para a
    // queue das rodas de forma a escrever no lcd a ação a realizar
    vTaskPrioritySet( xLCD, 3);
    // Faz um give de um semáforo para que a Task das Rodas possa correr
    xSemaphoreGive( xSemaforoRodas );

    if (var_infra > 7) {
      vTaskDelay(xWait);
    }

    /**********RECOMEÇAR O PROCESSO NOVAMENTE***********/
    if (var_infra == 12) {
      // espera até pressionar o botão de pressão
      while (digitalRead(robot.SWITCH1)) {};
      // espera até largar o botão de pressão
      while (!digitalRead(robot.SWITCH1)) {};
    }
    /***************************************************/
  }
}

/**************************************LCD*****************************************/
void LCD(void* pvParameters) {
  short int var = 0, command, aux, manter = 0;
  unsigned portBASE_TYPE uxPriority;
  portBASE_TYPE xStateQueueRodas;
  portBASE_TYPE xStateQueue;
  for (;;) {
    // Caso for a primeira vez que a Task LCD corra, imprime no LCD a seguinte informação
    // acerca do trabalho
    if (var == 0) {
      lcd.LcdString("Sistemas");
      lcd.gotoXY(1, 1);
      lcd.LcdString("Comp.");
      lcd.gotoXY(1, 2);

      lcd.LcdString("Embebidos");
      lcd.gotoXY(1, 3);
      lcd.LcdString("Trabalho1");
      delay(500);

      lcd.LcdClear();
      lcd.gotoXY(1, 0);
      lcd.LcdString("Bruno Silva");
      lcd.gotoXY(1, 1);
      lcd.LcdString("2150634");
      lcd.gotoXY(1, 2);
      lcd.LcdString("Samuel Neto");
      lcd.gotoXY(1, 3);
      lcd.LcdString("2150631");
      delay(500);

      lcd.LcdClear();
      lcd.gotoXY(1, 1);
      lcd.LcdString("Pressionar");
      lcd.gotoXY(1, 2);
      lcd.LcdString("Botao");

      while (digitalRead(robot.SWITCH1)) {};
      while (!digitalRead(robot.SWITCH1)) {};

      var = 1;

      // Diminui a prioridade da Task do LCD para voltar para a Task do Cérebro
      vTaskPrioritySet( NULL, 1);
    }

    // Verifica o valor colocado pelo cérebro na queue das rodas para poder escrever no lcd o
    // estado do robo correspondente
    xStateQueueRodas = xQueuePeek(xQueueRodas, &command, portMAX_DELAY);

    xStateQueue = xQueueReceive(xQueueLCD, &aux, 0);

    if (command == 0) {
      if (manter != 1) { // Se a variavel manter for diferente, isso significa que a informação que está
        // atualmente no LCD é diferente daquela que lá será colocada
        lcd.LcdClear();
        lcd.gotoXY(1, 1);
        lcd.LcdString("Contornar");
        lcd.gotoXY(1, 2);
        lcd.LcdString("Objeto");
        manter = 1;
      }
    } else if (aux == 1) {
      if (manter != 2) {
        lcd.LcdClear();
        lcd.gotoXY(20, 0);
        lcd.LCDBitmap(cruzamento);
        lcd.gotoXY(5, 4);
        lcd.LcdString("Intersecao");
        lcd.gotoXY(20, 5);
        lcd.LcdString("tipo 1");
        manter = 2;
      }
    } else if (aux == 2) {
      if (manter != 3) {
        lcd.LcdClear();
        lcd.gotoXY(20, 0);
        lcd.LCDBitmap(entrocamento_t);
        lcd.gotoXY(5, 4);
        lcd.LcdString("Intersecao");
        lcd.gotoXY(20, 5);
        lcd.LcdString("tipo 2");
        manter = 3;
      }
    } else if (aux == 3) {
      if (manter != 4) {
        lcd.LcdClear();
        lcd.gotoXY(20, 0);
        lcd.LCDBitmap(entrocamento_t90_E);
        lcd.gotoXY(5, 4);
        lcd.LcdString("Intersecao");
        lcd.gotoXY(20, 5);
        lcd.LcdString("tipo 3");
        manter = 4;
      }
    } else if (aux == 4) {
      if (manter != 5) {
        lcd.LcdClear();
        lcd.gotoXY(20, 0);
        lcd.LCDBitmap(entrocamento_t90_D);
        lcd.gotoXY(5, 4);
        lcd.LcdString("Intersecao");
        lcd.gotoXY(20, 5);
        lcd.LcdString("tipo 4");
        manter = 5;
      }
    } else if (aux == 5) {
      if (manter != 6) {
        lcd.LcdClear();
        lcd.gotoXY(5, 1);
        lcd.LcdString("A Procura");
        lcd.gotoXY(8, 2);
        lcd.LcdString("da Linha");
        manter = 6;
      }
    } else if (aux == 6) {
      if (manter != 7) {
        lcd.LcdClear();
        lcd.gotoXY(10, 1);
        lcd.LcdString("Voltar ao");
        lcd.gotoXY(13, 2);
        lcd.LcdString("Ponto de ");
        lcd.gotoXY(16, 3);
        lcd.LcdString("Partida");
        manter = 7;
      }
    } else if (aux == 7) {
      if (manter != 8) {
        lcd.LcdClear();
        lcd.gotoXY(17, 2);
        lcd.LcdString("Parado!");
        manter = 8;
      }
    } else {
      if (manter != 9) {
        lcd.LcdClear();
        lcd.gotoXY(8, 1);
        lcd.LcdString("Seguir");
        lcd.gotoXY(10, 2);
        lcd.LcdString("Linha");
        manter = 9;
      }
    }

    // Diminui a prioridade da Task do LCD para voltar para a Task do Cérebro
    vTaskPrioritySet( NULL, 1);

  }
}

void loop() {}


