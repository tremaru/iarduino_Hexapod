#include "iarduino_MultiServo.h"
#include "EEPROM.h"
#include "iarduino_Hexapod.h"

		iarduino_Hexapod::iarduino_Hexapod(uint8_t i, uint8_t j, uint8_t k){
			iXP_pins_ButtonA=i;	if(iXP_pins_ButtonA<255)						{pinMode(iXP_pins_ButtonA, INPUT);}
			iXP_pins_ButtonB=j;	if(iXP_pins_ButtonB<255)						{pinMode(iXP_pins_ButtonB, INPUT);}
			iXP_pins_LED=k;		if(iXP_pins_ButtonA<255 && iXP_pins_ButtonB<255){pinMode(iXP_pins_LED, OUTPUT); digitalWrite(iXP_pins_LED, LOW);}
		}

//		инициализация HEXAPOD
void	iarduino_Hexapod::begin(uint8_t i){										//	(тип сервоприводов)
			iXP_MSS.servoSet(SERVO_ALL, i);										//  Позиционирование сервоприводов (все сервоприводы , название сервоприводов)
			iXP_MSS.begin();													//  Инициализация модуля (адрес по умолчанию = 0x40  , частота по умолчанию = 50Гц)
			iXP_flag_Calibration=iXP_func_ReadAngleCenter();					//	Чтение сохранённых значений центрального угла, для каждого сустава, каждой конечности
		}

//		калибровка HEXAPOD
void	iarduino_Hexapod::calibration(){ if(iXP_pins_ButtonA<255 && iXP_pins_ButtonB<255){
			iXP_time_ButtonA=0;																			//	Время удержания кнопки A (в сотых долях секунды)
			iXP_time_ButtonB=0;																			//	Время удержания кнопки B (в сотых долях секунды)
			uint8_t f=0;																				//	Флаг указывающий на то, что зафиксировано удержание/нажатие кнопки
			if(iXP_step_Calibration){iXP_flag_Calibration=true;}										//	Включаем работу всех функций библиотеки
//			Удержание кнопки (кнопок)
			while(digitalRead(iXP_pins_ButtonA)||digitalRead(iXP_pins_ButtonB)){						//	Если нажата кнопка A и/или кнопка B, то cоздаём цикл, пока кнопка(и) не будет(ут) отпушена(ы)
				if(digitalRead(iXP_pins_ButtonA) && iXP_time_ButtonA<65535){iXP_time_ButtonA++;}		//	Если удерживается кнопка A, то увеличиваем время её удержания
				if(digitalRead(iXP_pins_ButtonB) && iXP_time_ButtonB<65535){iXP_time_ButtonB++;}		//	Если удерживается кнопка B, то увеличиваем время её удержания
//				Если удерживаются обе кнопки, дольше чем iXP_time_ButtonDelay мс
				if(!f && iXP_time_ButtonA>=iXP_time_ButtonDelay/10 && iXP_time_ButtonB>=iXP_time_ButtonDelay/10){ f=1;
					if(iXP_step_Calibration==0){														//	Если калибровка не активна, то ...
						iXP_step_Calibration=1;															//	Переход к установке вертикальных суставов конечностей
						iXP_flag_Calibration=true;														//	Включаем работу всех функций библиотеки
						set_motion(SERVO_ALL, 1, SERVO_FREE);											//	Ослабляем вертикальные суставы всех конечностей
						set_motion(SERVO_ALL, 2, SERVO_FREE);											//	Ослабляем горизонтальные суставы всех конечностей
					}else if(iXP_step_Calibration>=1  && iXP_step_Calibration<=2 ){						//	Если установлены вертикальные суставы конечностей, то ...
						iXP_step_Calibration=3;															//	Переход к центрированию вертикальных суставов конечностей
						set_motion(SERVO_ALL, 1, 50);													//	Поднимаем вертикальные суставы всех конечностей в условно центральное положение
						set_motion(SERVO_ALL, 2, SERVO_FREE);											//	Ослабляем горизонтальные суставы всех конечностей
					}else if(iXP_step_Calibration>=3  && iXP_step_Calibration<=9 ){						//	Если отцентрированны вертикальные суставы конечностей, то ...
						iXP_step_Calibration=10;														//	Переход к центрированию горизонтальных суставов конечностей
						set_motion(SERVO_ALL, 1, 0);													//	Опускаем вертикальные суставы всех конечностей
						set_motion(SERVO_ALL, 2, SERVO_FREE);											//	Ослабляем горизонтальные суставы всех конечностей
					}else if(iXP_step_Calibration>=10 && iXP_step_Calibration<=16){						//	Если отцентрированны горизонтальные суставы конечностей, то ...
						for(int i=0; i<= 5; i++){
							EEPROM.write(iXP_addr_EEPROM+i,   iXP_angle_Hor_Center[i]);					//	Сохраняем центральное положение горизонтальных суставов
							EEPROM.write(iXP_addr_EEPROM+i+6, iXP_angle_Ver_Center[i]);					//	Сохраняем центральное положение вертикальных   суставов
						}	EEPROM.write(iXP_addr_EEPROM+12,  0xAA);									//	Сохраняем константу 0xAA в последнюю выделенную ячейку памяти
						iXP_step_Calibration=0;															//	Выход из калибровки
						iXP_flag_Calibration=iXP_func_ReadAngleCenter();								//	Чтение сохранённых значений центрального угла, для каждого сустава, каждой конечности
						go(COM_TEST);																	//	Выполняем команду ТЕСТ
					}
//				Если удерживается только кнопка A
				}else if(!f && iXP_time_ButtonA>=iXP_time_ButtonDelay/10 && iXP_time_ButtonB==0){ f=1;
					if(iXP_step_Calibration>=3 && iXP_step_Calibration<=9){								//	Если центрируются вертикальные суставы конечностей, то ...
						iXP_step_Calibration++; if(iXP_step_Calibration>9){iXP_step_Calibration=4;}		//	Увеличиваем номер режима калибровки, но не позволяем выйти за предел (9)
						set_motion(1, 1, (iXP_step_Calibration==4?100:50));								//	Поднимаем или опускаем вертикальный сустав 1 конечности
						set_motion(2, 1, (iXP_step_Calibration==5?100:50));								//	Поднимаем или опускаем вертикальный сустав 2 конечности
						set_motion(3, 1, (iXP_step_Calibration==6?100:50));								//	Поднимаем или опускаем вертикальный сустав 3 конечности
						set_motion(4, 1, (iXP_step_Calibration==7?100:50));								//	Поднимаем или опускаем вертикальный сустав 4 конечности
						set_motion(5, 1, (iXP_step_Calibration==8?100:50));								//	Поднимаем или опускаем вертикальный сустав 5 конечности
						set_motion(6, 1, (iXP_step_Calibration==9?100:50));								//	Поднимаем или опускаем вертикальный сустав 6 конечности
					}else if(iXP_step_Calibration>=10 && iXP_step_Calibration<=16){						//	Если центрируются горизонтальные суставы конечностей, то ...
						iXP_step_Calibration++; if(iXP_step_Calibration>16){iXP_step_Calibration=11;}	//	Увеличиваем номер режима калибровки, но не позволяем выйти за предел (16)
						set_motion(1, 1, (iXP_step_Calibration==11?100:0));								//	Поднимаем или опускаем вертикальный сустав 1 конечности
						set_motion(2, 1, (iXP_step_Calibration==12?100:0));								//	Поднимаем или опускаем вертикальный сустав 2 конечности
						set_motion(3, 1, (iXP_step_Calibration==13?100:0));								//	Поднимаем или опускаем вертикальный сустав 3 конечности
						set_motion(4, 1, (iXP_step_Calibration==14?100:0));								//	Поднимаем или опускаем вертикальный сустав 4 конечности
						set_motion(5, 1, (iXP_step_Calibration==15?100:0));								//	Поднимаем или опускаем вертикальный сустав 5 конечности
						set_motion(6, 1, (iXP_step_Calibration==16?100:0));								//	Поднимаем или опускаем вертикальный сустав 6 конечности
						set_motion(SERVO_ALL, 2, SERVO_FREE);											//	Ослабляем горизонтальные суставы всех конечностей
						set_motion((iXP_step_Calibration-10), 2, 50);									//	Устанавливаем горизонтальный сустав центрируемой конечности в центральное положение
					}
//				Если удерживается только кнопка B
				}else if(!f && iXP_time_ButtonB>=iXP_time_ButtonDelay/10 && iXP_time_ButtonA==0){ f=1;
					if(iXP_step_Calibration>=3 && iXP_step_Calibration<=9){								//	Если центрируются вертикальные суставы конечностей, то ...
						iXP_step_Calibration--; if(iXP_step_Calibration<4){iXP_step_Calibration=9;}		//	Уменьшаем номер режима калибровки, но не позволяем выйти за предел (3)
						set_motion(1, 1, (iXP_step_Calibration==4?100:50));								//	Поднимаем или опускаем вертикальный сустав 1 конечности
						set_motion(2, 1, (iXP_step_Calibration==5?100:50));								//	Поднимаем или опускаем вертикальный сустав 2 конечности
						set_motion(3, 1, (iXP_step_Calibration==6?100:50));								//	Поднимаем или опускаем вертикальный сустав 3 конечности
						set_motion(4, 1, (iXP_step_Calibration==7?100:50));								//	Поднимаем или опускаем вертикальный сустав 4 конечности
						set_motion(5, 1, (iXP_step_Calibration==8?100:50));								//	Поднимаем или опускаем вертикальный сустав 5 конечности
						set_motion(6, 1, (iXP_step_Calibration==9?100:50));								//	Поднимаем или опускаем вертикальный сустав 6 конечности
					}else if(iXP_step_Calibration>=10 && iXP_step_Calibration<=16){						//	Если центрируются горизонтальные суставы конечностей, то ...
						iXP_step_Calibration--; if(iXP_step_Calibration<11){iXP_step_Calibration=16;}	//	Уменьшаем номер режима калибровки, но не позволяем выйти за предел (10)
						set_motion(1, 1, (iXP_step_Calibration==11?100:0));								//	Поднимаем или опускаем вертикальный сустав 1 конечности
						set_motion(2, 1, (iXP_step_Calibration==12?100:0));								//	Поднимаем или опускаем вертикальный сустав 2 конечности
						set_motion(3, 1, (iXP_step_Calibration==13?100:0));								//	Поднимаем или опускаем вертикальный сустав 3 конечности
						set_motion(4, 1, (iXP_step_Calibration==14?100:0));								//	Поднимаем или опускаем вертикальный сустав 4 конечности
						set_motion(5, 1, (iXP_step_Calibration==15?100:0));								//	Поднимаем или опускаем вертикальный сустав 5 конечности
						set_motion(6, 1, (iXP_step_Calibration==16?100:0));								//	Поднимаем или опускаем вертикальный сустав 6 конечности
						set_motion(SERVO_ALL, 2, SERVO_FREE);											//	Ослабляем горизонтальные суставы всех конечностей
						set_motion((iXP_step_Calibration-10), 2, 50);									//	Устанавливаем горизонтальный сустав центрируемой конечности в центральное положение
					}
				}
				iXP_func_CalibrationLed();																//	Управляем светодиодом
				delay(10);																				//  Пропускаем 0,01с
			}
//			Если калибровка активна, то ...
			if(iXP_step_Calibration){
//				Выполняем команды после отпускания ужерживаемых кнопок
				if(f){
					if(iXP_step_Calibration>=4 && iXP_step_Calibration<=9){								//	Если центрируются вертикальные суставы конечностей, то ...
						set_motion(SERVO_ALL, 1, 50);													//	Поднимаем вертикальные суставы всех конечностей в условно центральное положение
					}
				}
//				Если нажата и отпущена (без удержаний) только кнопка A
				if(!f && iXP_time_ButtonA>0 && iXP_time_ButtonB==0){ f=1;
					if(iXP_step_Calibration==1){														//	Если устанавливаются вертикальные суставы конечностей, то ...
						iXP_step_Calibration=2;															//	Переходим к подниманию всех конечностей
						iXP_MSS.servoWrite(iXP_func_JointPin(1,1),iXP_angle_Odd_Calibration);			//	Поднимаем вертикальный сустав 1 конечности на угол iXP_angle_Odd_Calibration
						iXP_MSS.servoWrite(iXP_func_JointPin(3,1),iXP_angle_Odd_Calibration);			//	Поднимаем вертикальный сустав 3 конечности на угол iXP_angle_Odd_Calibration
						iXP_MSS.servoWrite(iXP_func_JointPin(5,1),iXP_angle_Odd_Calibration);			//	Поднимаем вертикальный сустав 5 конечности на угол iXP_angle_Odd_Calibration
						iXP_MSS.servoWrite(iXP_func_JointPin(2,1),iXP_angle_Evn_Calibration);			//	Поднимаем вертикальный сустав 2 конечности на угол iXP_angle_Evn_Calibration
						iXP_MSS.servoWrite(iXP_func_JointPin(4,1),iXP_angle_Evn_Calibration);			//	Поднимаем вертикальный сустав 4 конечности на угол iXP_angle_Evn_Calibration
						iXP_MSS.servoWrite(iXP_func_JointPin(6,1),iXP_angle_Evn_Calibration);			//	Поднимаем вертикальный сустав 6 конечности на угол iXP_angle_Evn_Calibration
						set_motion(SERVO_ALL, 2, SERVO_FREE);											//	Ослабляем горизонтальные суставы всех конечностей
					}else if(iXP_step_Calibration>=4 && iXP_step_Calibration<=9){						//	Если центрируются вертикальные суставы конечностей, то ...
						iXP_angle_Ver_Center[(iXP_step_Calibration-4)]+=iXP_step_Calibration%2?1:-1;	//	Сдвигаем центральный угол вертикального сустава, центрируемой конечности, вверх
						set_motion((iXP_step_Calibration-3), 1, 50);									//	Устанавливаем вертикальный сустав центрируемой конечности в центральное положение
					}else if(iXP_step_Calibration>=11 && iXP_step_Calibration<=16){						//	Если центрируются горизонтальные суставы конечностей, то ...
						iXP_angle_Hor_Center[(iXP_step_Calibration-11)]+=iXP_step_Calibration%2?-1:1;	//	Сдвигаем центральный угол горизонтального сустава, центрируемой конечности, вперед
						set_motion((iXP_step_Calibration-10), 2, 50);									//	Устанавливаем горизонтальный сустав центрируемой конечности в центральное положение
					}
//				Если нажата и отпущена (без удержаний) только кнопка B
				}else if(!f && iXP_time_ButtonB>0 && iXP_time_ButtonA==0){ f=1;
					if(iXP_step_Calibration==2){														//	Если устанавливаются вертикальные суставы конечностей, то ...
						iXP_step_Calibration=1;															//	Переходим к ослаблению всех конечностей
						set_motion(SERVO_ALL, 1, SERVO_FREE);											//	Ослабляем вертикальные суставы всех конечностей
						set_motion(SERVO_ALL, 2, SERVO_FREE);											//	Ослабляем горизонтальные суставы всех конечностей
					}else if(iXP_step_Calibration>=4 && iXP_step_Calibration<=9){						//	Если центрируются вертикальные суставы конечностей, то ...
						iXP_angle_Ver_Center[(iXP_step_Calibration-4)]+=iXP_step_Calibration%2?-1:1;	//	Сдвигаем центральный угол вертикального сустава, центрируемой конечности, вниз
						set_motion((iXP_step_Calibration-3), 1, 50);									//	Устанавливаем вертикальный сустав центрируемой конечности в центральное положение
					}else if(iXP_step_Calibration>=11 && iXP_step_Calibration<=16){						//	Если центрируются горизонтальные суставы конечностей, то ...
						iXP_angle_Hor_Center[(iXP_step_Calibration-11)]+=iXP_step_Calibration%2?1:-1;	//	Сдвигаем центральный угол горизонтального сустава, центрируемой конечности, назад
						set_motion((iXP_step_Calibration-10), 2, 50);									//	Устанавливаем горизонтальный сустав центрируемой конечности в центральное положение
					}
				}
				iXP_func_CalibrationLed();																//	Управляем светодиодом
				iXP_flag_Calibration=false;																//	Отключаем работу всех функций библиотеки
			}
		}}

//		сброс калибровки
void	iarduino_Hexapod::calibration_reset(){
			for(int i=0; i<=12; i++){
				EEPROM.write(iXP_addr_EEPROM+i, 255);
			}
		}

//		движение по константам
void	iarduino_Hexapod::iXP_func_Go(uint8_t i, uint8_t j){											//	(команда, скорость/параметр)
//			если HEXAPOD откалиброван
			if(iXP_flag_Calibration){
//				корректируем ошибки при вводе параметров
				if(j==255){j=4;} if(j > 10){j = 10;}
				switch(i){
				/* движение вперед	*/	case GO_ON:			go( 0, j);																		break;
				/* движение назад	*/	case GO_BACK:		go( 0,-j);																		break;
				/* поворот  влево	*/	case GO_LEFT:		go( j, 0);																		break;
				/* поворот  вправо	*/	case GO_RIGHT:		go(-j, 0);																		break;
				/* движение с повор	*/	case GO_ON_LEFT:	go( 4, j);																		break;
				/* движение с повор	*/	case GO_ON_RIGHT:	go(-4, j);																		break;
				/* движение с повор	*/	case GO_BACK_LEFT:	go( 4,-j);																		break;
				/* движение с повор	*/	case GO_BACK_RIGHT:	go(-4,-j);																		break;
				/* команда  спать	*/	case COM_SLEEP:		set_motion(SERVO_ALL, 1, SERVO_FREE);	set_motion(SERVO_ALL, 2, SERVO_FREE);	break;
				/* команда  стоп	*/	case COM_STOP:		go( 0, 0, 0);							set_motion(1, 2, 100);
																									set_motion(2, 2, 100);
																									set_motion(3, 2, 50);
																									set_motion(4, 2, 50);
																									set_motion(5, 2, 0);
																									set_motion(6, 2, 0);					break;
				/* команда  сидеть	*/	case COM_SIT:		set_motion(1, 1, 100);					set_motion(1, 2, 100);
															set_motion(2, 1, 100);					set_motion(2, 2, 100);
															set_motion(3, 1, 50);					set_motion(3, 2, 50);
															set_motion(4, 1, 50);					set_motion(4, 2, 50);
															set_motion(5, 1, 0);					set_motion(5, 2, 0);
															set_motion(6, 1, 0);					set_motion(6, 2, 0);					break;
				/* команда  поклон	*/	case COM_NOD:		set_motion(1, 1, 0);					set_motion(1, 2, 100);
															set_motion(2, 1, 0);					set_motion(2, 2, 100);
															set_motion(3, 1, 50);					set_motion(3, 2, 50);
															set_motion(4, 1, 50);					set_motion(4, 2, 50);
															set_motion(5, 1, 100);					set_motion(5, 2, 0);
															set_motion(6, 1, 100);					set_motion(6, 2, 0);					break;
				/* команда  встать	*/	case COM_UP:		go( 0, 0, j);																	break;
				/* команда  лежать	*/	case COM_LAY:		go( 0, 0,-j);																	break;
				/* команда  тест	*/	case COM_TEST:		set_motion(SERVO_ALL, 2, 50);
															set_motion(SERVO_ALL, 1, 50);
																for(int b=50;  b>=0;   b--){set_motion(SERVO_ALL, 1, b);}
																for(int b=0;   b<=100; b++){set_motion(SERVO_ALL, 1, b);}
																for(int b=100; b>=50;  b--){set_motion(SERVO_ALL, 1, b);}
															for(int a=1;   a<=6;   a++){
																for(int b=50;  b<=100; b++){set_motion(a, 1, b); delay(1);}
																for(int b=50;  b<=100; b++){set_motion(a, 2, b); delay(1);}
																for(int b=100; b>=0;   b--){set_motion(a, 2, b); delay(1);}
																for(int b=0;   b<=100; b++){set_motion(a, 2, b); delay(1);}
																for(int b=100; b>=0;   b--){set_motion(a, 2, b); delay(1);}
																for(int b=0;   b<=50;  b++){set_motion(a, 2, b); delay(1);}
																for(int b=100; b>=50;  b--){set_motion(a, 1, b); delay(1);}
															}																				break;
				}
			}
		}

//		движение по координатам XY
void	iarduino_Hexapod::go(int i, int j, int k){						//	(-10 лево / +10 право , -10 назад / +10 вперед , -10 низ / +10 верх)
//			если HEXAPOD откалиброван
			if(iXP_flag_Calibration){
				uint8_t	l = iXP_limit_Ver;
//				перенаправляем вызов на функцию iXP_func_Go
				if(i>=100 && i<=255){
					if(j==255){j=4;} if(j>10){j=10;}
					iXP_func_Go(i,j); return;
				}
//				корректируем ошибки при вводе параметров
							if(i <-10){i =-10;} if(i > 10){i = 10;}
							if(j <-10){j =-10;} if(j > 10){j = 10;}
				if(k!=255){	if(k <-10){k =-10;} if(k > 10){k = 10;} }
//				устанавливаем ограничения
							iXP_limit_Hor_Odd	= 0; if(i>0){iXP_limit_Hor_Odd = i*10;}		//	ограничение по горизонтали 0 - 100% (для нечетных конечностей)
							iXP_limit_Hor_Evn	= 0; if(i<0){iXP_limit_Hor_Evn =-i*10;}		//	ограничение по горизонтали 0 - 100% (для   четных конечностей)
				if(k!=255){	iXP_limit_Ver		= map(k, -10, 10, 100, 0);}					//	ограничение по высоте      0 - 100%
//				изменение высоты
				if(k!=255 && i==0 && j==0 && l!=iXP_limit_Ver){set_motion(SERVO_ALL, 1, iXP_limit_Ver);}
//				поворот на месте
				if(i!=0 && j==0){
//					устанавливаем скорость
					iXP_gogo_360+=i*2;
					if(iXP_gogo_360 >=360){iXP_gogo_360=0;  }
					if(iXP_gogo_360 < 0  ){iXP_gogo_360=360;}
//					совершаем движение
					set_motionA(1,     iXP_gogo_360    , true);					//		0	>	>	>	>	>	>	>	>	>	> <<<<<	360		|		|		|		|		|
					set_motionA(4, 360-iXP_gogo_360+240, true);					//		|		60	<	<	<	<	<	<	<	<	<	< >>>>>	420		|		|		|		|
					set_motionA(5, 120+iXP_gogo_360    , true);					//		|		|		120	>	>	>	>	>	>	>	>	>	< <<<<<	480		|		|		|
					set_motionA(2, 360-iXP_gogo_360+120, true);					//		|		|		|		180	<	<	<	<	<	<	<	<	<	< >>>>>	540		|		|
					set_motionA(3, 240+iXP_gogo_360    , true);					//		|		|		|		|		240	>	>	>	>	>	>	>	>	>	> <<<<<	600		|
					set_motionA(6, 360-iXP_gogo_360    , true);					//		|		|		|		|		|		300	<	<	<	<	<	<	<	<	<	< >>>>>	660
//				движение по градусу
				}else if(i!=0 || j!=0){
//					устанавливаем скорость
					iXP_gogo_360+=j*2;
					if(iXP_gogo_360 >=360){iXP_gogo_360=0;  }
					if(iXP_gogo_360 < 0  ){iXP_gogo_360=360;}
//					совершаем движение
					switch(walking){											//	тип походки
						case 1:
						set_motionA(1,     iXP_gogo_360, true, true);			//		0	>	>	>	>	>	>	>	>	>	> <<<<<	360		|		|		|		|		|
						set_motionA(2, 60 +iXP_gogo_360, true, true);			//		|		60	>	>	>	>	>	>	>	>	>	> <<<<<	420		|		|		|		|
						set_motionA(3, 120+iXP_gogo_360, true, true);			//		|		|		120	>	>	>	>	>	>	>	>	>	> <<<<<	480		|		|		|
						set_motionA(4, 180+iXP_gogo_360, true, true);			//		|		|		|		180	>	>	>	>	>	>	>	>	>	> <<<<<	540		|		|
						set_motionA(5, 240+iXP_gogo_360, true, true);			//		|		|		|		|		240	>	>	>	>	>	>	>	>	>	> <<<<<	600		|
						set_motionA(6, 300+iXP_gogo_360, true, true);			//		|		|		|		|		|		300	>	>	>	>	>	>	>	>	>	> <<<<<	660
						break;
						case 2:
						set_motionA(1,     iXP_gogo_360, true, true);			//		0	>	>	>	>	>	>	>	>	>	> <<<<<	360		|		|		|		|		|
						set_motionA(3, 60 +iXP_gogo_360, true, true);			//		|		60	>	>	>	>	>	>	>	>	>	> <<<<<	420		|		|		|		|
						set_motionA(5, 120+iXP_gogo_360, true, true);			//		|		|		120	>	>	>	>	>	>	>	>	>	> <<<<<	480		|		|		|
						set_motionA(2, 180+iXP_gogo_360, true, true);			//		|		|		|		180	>	>	>	>	>	>	>	>	>	> <<<<<	540		|		|
						set_motionA(4, 240+iXP_gogo_360, true, true);			//		|		|		|		|		240	>	>	>	>	>	>	>	>	>	> <<<<<	600		|
						set_motionA(6, 300+iXP_gogo_360, true, true);			//		|		|		|		|		|		300	>	>	>	>	>	>	>	>	>	> <<<<<	660
						break;
						case 3:
						set_motionA(1,     iXP_gogo_360, true, true);			//		0	>	>	>	>	>	>	>	>	>	> <<<<<	360		|		|		|		|		|
						set_motionA(4, 60 +iXP_gogo_360, true, true);			//		|		60	>	>	>	>	>	>	>	>	>	> <<<<<	420		|		|		|		|
						set_motionA(5, 120+iXP_gogo_360, true, true);			//		|		|		120	>	>	>	>	>	>	>	>	>	> <<<<<	480		|		|		|
						set_motionA(2, 180+iXP_gogo_360, true, true);			//		|		|		|		180	>	>	>	>	>	>	>	>	>	> <<<<<	540		|		|
						set_motionA(3, 240+iXP_gogo_360, true, true);			//		|		|		|		|		240	>	>	>	>	>	>	>	>	>	> <<<<<	600		|
						set_motionA(6, 300+iXP_gogo_360, true, true);			//		|		|		|		|		|		300	>	>	>	>	>	>	>	>	>	> <<<<<	660
						break;
						default:
						set_motionA(1,     iXP_gogo_360, true, true);			//		0	>	>	>	>	>	>	>	>	>	> <<<<<	360		|		|		|		|		|
						set_motionA(2,     iXP_gogo_360, true, true);			//		0	>	>	>	>	>	>	>	>	>	> <<<<<	360		|		|		|		|		|
						set_motionA(3,     iXP_gogo_360, true, true);			//		0	>	>	>	>	>	>	>	>	>	> <<<<<	360		|		|		|		|		|
						set_motionA(4,     iXP_gogo_360, true, true);			//		0	>	>	>	>	>	>	>	>	>	> <<<<<	360		|		|		|		|		|
						set_motionA(5,     iXP_gogo_360, true, true);			//		0	>	>	>	>	>	>	>	>	>	> <<<<<	360		|		|		|		|		|
						set_motionA(6,     iXP_gogo_360, true, true);			//		0	>	>	>	>	>	>	>	>	>	> <<<<<	360		|		|		|		|		|
					}
				}
			}
		}

//		движение одной или всеми конечностями
void	iarduino_Hexapod::set_motion(uint8_t i, uint8_t j, uint8_t k){			//	(номер конечности: 1-6 / 255 , номер сустава: 1=V / 2=H , положение сустава: внизу-сзади 0...50...100 вверху-спереди / 255)
//			если HEXAPOD откалиброван
			if(iXP_flag_Calibration){
//				корректируем ошибки при вводе параметров
				if(i==0){i=1;} if(j==0){j=1;} if(j>2){j=2;} if(i>6&& i!=SERVO_ALL){i=6;} if(k>100 && k!=SERVO_FREE){k=100;}
//				если номер конечности == SERVO_ALL, то требуется пройти по всем конечностям
				uint8_t a=i, b=i; if(i==SERVO_ALL){a=1; b=6;}					//	применить к конечностям: от a до b
//				проходим по указанным конечностям
				for(int n=a; n<=b; n++){
					if(k==SERVO_FREE)	{iXP_MSS.digitalWrite(iXP_func_JointPin(n,j),LOW);}
					else				{iXP_MSS.servoWrite  (iXP_func_JointPin(n,j),(j==1?iXP_func_AngleVer(n,k):iXP_func_AngleHor(n,k)));}
				}
			}
		}

//		текущее положение одной конечности
//void	iarduino_Hexapod::get_motion(uint8_t,uint8_t){							//	(номер конечности: 1-6 , номер сустава: 1=V / 2=H)
//			
//		}

//		движение одной или всеми конечностями автоматически
void	iarduino_Hexapod::set_motionA(uint8_t i, int j, bool f1, bool f2){		//	(номер конечности: 1-6 / 255 , градус движения: 1-360 , учитывать ограничение по вертикали: iXP_limit_Ver % , учитывать ограничение по горизонтали: iXP_limit_Hor % )
//			если HEXAPOD откалиброван
			if(iXP_flag_Calibration){
//				корректируем введённые параметры
				if(i==0){i=1;} if(i>6 && i!=SERVO_ALL){i=6;} if(j<0){j*=-1;} if(j>=360){j=j%360;}
//				если номер конечности == SERVO_ALL, то требуется пройти по всем конечностям
				uint8_t a=i, b=i; if(i==SERVO_ALL){a=1; b=6;}						//	применить к конечностям: от a до b
				int		l1, l2;														//	вычисляемое, вертикальное l1 и горизонтальное l2, положение конечностей (0-100)
//				проходим по указанным конечностям
				for(int n=a; n<=b; n++){
//					вычисляем горизотальное положение
					if(j< 300)	{l2=map(j,   0, 300, 100,   0);	}else				//	прямой   ход (100 >>> 0)
								{l2=map(j, 300, 360, 0  , 100);	}					//	обратный ход (100 <<< 0)
//					вычисляем вертикальное положение
					if(j< 300)	{l1=0;							}else				//	конченость опущена
					if(j<=310)	{l1=map(j, 300, 310,   0, 100);	}else				//	конечность опускается
					if(j>=350)	{l1=map(j, 350, 360, 100,   0);	}else				//	конечность поднимается
								{l1=100;						}					//	конечность поднята
//					применяем установленные ограничения
					if(f2)		{l2=map(l2, 0, 100, (n%2?iXP_limit_Hor_Evn:iXP_limit_Hor_Odd)/2, 100-((n%2?iXP_limit_Hor_Evn:iXP_limit_Hor_Odd)/2));}
					if(f1)		{l1=map(l1, 0, 100, iXP_limit_Ver, 100);}
//					применяем положение конечности
					set_motion(n, 2, l2);
					set_motion(n, 1, l1);
				}
			}
		}

//		определение номера вывода по номеру конечности и сустава
uint8_t	iarduino_Hexapod::iXP_func_JointPin(uint8_t i,uint8_t j){				//	(номер конечности 1-6 , номер сустава 1=V / 2=H)
				 if(i==1 && j==1){return JOINT_1V;}
			else if(i==1 && j==2){return JOINT_1H;}
			else if(i==2 && j==1){return JOINT_2V;}
			else if(i==2 && j==2){return JOINT_2H;}
			else if(i==3 && j==1){return JOINT_3V;}
			else if(i==3 && j==2){return JOINT_3H;}
			else if(i==4 && j==1){return JOINT_4V;}
			else if(i==4 && j==2){return JOINT_4H;}
			else if(i==5 && j==1){return JOINT_5V;}
			else if(i==5 && j==2){return JOINT_5H;}
			else if(i==6 && j==1){return JOINT_6V;}
			else if(i==6 && j==2){return JOINT_6H;}
			else				 {return SERVO_ALL;}
		}

//		определение угла вертикального сустава по номеру конечности и положению
uint8_t	iarduino_Hexapod::iXP_func_AngleVer(uint8_t i, uint8_t j){				//	(номер конечности 1-6 , положение вертикального сустава: внизу 0...50...100 вверху)
			int a = iXP_angle_Ver_Center[i-1],	b = iXP_angle_Ver_Center[i-1];	//	центральный угол по вертикали
			if(i%2)	{//	для нечётных конечностей								//	1,3,5
				a+= iXP_angle_Odd_Bottom;		b-= iXP_angle_Odd_Top;			//	± максимальный угол смещения по вертикали
				a+= 50 - JOINT_VMIN[i-1];		b+= 50 - JOINT_VMAX[i-1];		//	± корректировка крайнего положения
			}else	{//	для чётных конечностей									//	2,4,6
				a-= iXP_angle_Evn_Bottom;		b+= iXP_angle_Evn_Top;			//	± максимальный угол смещения по вертикали
				a+= JOINT_VMIN[i-1] - 50;		b+= JOINT_VMAX[i-1] - 50;		//	± корректировка крайнего положения
			}		//	корректировка полученных значений
			if(a<0){a=0;} if(b<0){b=0;} if(a>255){a=255;} if(b>255){b=255;} return map(j,0,100,a,b);
		}
		
//		определение угла горизонтального сустава по номеру конечности и положению
uint8_t	iarduino_Hexapod::iXP_func_AngleHor(uint8_t i, uint8_t j){				//	(номер конечности 1-6 , положение горизонтального сустава: сзади 0...50...100 спереди)
			int a = iXP_angle_Hor_Center[i-1],	b = iXP_angle_Hor_Center[i-1];	//	центральный угол по горизонтали
			if(i%2)	{//	для нечётных конечностей								//	1,3,5
				a+= iXP_angle_Odd_Back;			b-= iXP_angle_Odd_Ahead ;		//	± максимальный угол смещения по горизонтали
				a+= 50 - JOINT_HMIN[i-1];		b+= 50 - JOINT_HMAX[i-1];		//	± корректировка крайнего положения
			}else	{//	для чётных конечностей									//	2,4,6
				a-= iXP_angle_Evn_Back;			b+= iXP_angle_Evn_Ahead ;		//	± максимальный угол смещения по горизонтали
				a+= JOINT_HMIN[i-1] - 50;		b+= JOINT_HMAX[i-1] - 50;		//	± корректировка крайнего положения
			}		//	корректировка полученных значений
			if(a<0){a=0;} if(b<0){b=0;} if(a>255){a=255;} if(b>255){b=255;} return map(j,0,100,a,b);
		}
		
//		чтение сохранённых значений центрального угла, для каждого сустава, каждой конечности
bool	iarduino_Hexapod::iXP_func_ReadAngleCenter(){
			bool j=true;
				for(int i=0; i<=11; i++){
					if (EEPROM.read(iXP_addr_EEPROM+i )<   10){j=false;}
					if (EEPROM.read(iXP_addr_EEPROM+i )>  170){j=false;}
				}	if (EEPROM.read(iXP_addr_EEPROM+12)!=0xAA){j=false;}
			if(j){
				for(int i=0; i<= 5; i++){
					iXP_angle_Hor_Center[i]=EEPROM.read(iXP_addr_EEPROM+i);
					iXP_angle_Ver_Center[i]=EEPROM.read(iXP_addr_EEPROM+i+6);
				}
			}else{digitalWrite(iXP_pins_LED, HIGH);}
				return j;
		}

//		управление светодиодом во время калибровки
void	iarduino_Hexapod::iXP_func_CalibrationLed(){
//			если HEXAPOD откалиброван или калибруется
			if(iXP_flag_Calibration){
				int a, b;														//	время удержания светодиода в положении: a-вкл, b-выкл
				switch(iXP_step_Calibration){									//	шаг калибровки:
					case  0:	a=0;	b=5000;	break;							//	нет
					case  1:	a=500;	b=500;	break;							//	установка по вертикали,   все конечности ослаблены
					case  2:	a=50;	b=50;	break;							//	установка по вертикали,   все конечности подняты

					case  3:	a=100;	b=1200;	break;							//	установка по вертикали,   все конечности свободны по горизонтали и установлены в центр по вертикали
					case  4:	a=100;	b=500;	break;							//	установка по вертикали,   все конечности свободны по горизонтали и установлены в центр по вертикали, центрируется 1 конечность
					case  5:	a=100;	b=500;	break;							//	установка по вертикали,   все конечности свободны по горизонтали и установлены в центр по вертикали, центрируется 1 конечность
					case  6:	a=100;	b=500;	break;							//	установка по вертикали,   все конечности свободны по горизонтали и установлены в центр по вертикали, центрируется 1 конечность
					case  7:	a=100;	b=500;	break;							//	установка по вертикали,   все конечности свободны по горизонтали и установлены в центр по вертикали, центрируется 1 конечность
					case  8:	a=100;	b=500;	break;							//	установка по вертикали,   все конечности свободны по горизонтали и установлены в центр по вертикали, центрируется 1 конечность
					case  9:	a=100;	b=500;	break;							//	установка по вертикали,   все конечности свободны по горизонтали и установлены в центр по вертикали, центрируется 1 конечность

					case 10:	a=1200;	b=100;	break;							//	установка по горизонтали, все конечности опущены и свободны по горизонтали
					case 11:	a=500;	b=100;	break;							//	установка по горизонтали, все конечности опущены и свободны по горизонтали, кроме 1, которая поднята и зафиксирована
					case 12:	a=500;	b=100;	break;							//	установка по горизонтали, все конечности опущены и свободны по горизонтали, кроме 2, которая поднята и зафиксирована
					case 13:	a=500;	b=100;	break;							//	установка по горизонтали, все конечности опущены и свободны по горизонтали, кроме 3, которая поднята и зафиксирована
					case 14:	a=500;	b=100;	break;							//	установка по горизонтали, все конечности опущены и свободны по горизонтали, кроме 4, которая поднята и зафиксирована
					case 15:	a=500;	b=100;	break;							//	установка по горизонтали, все конечности опущены и свободны по горизонтали, кроме 5, которая поднята и зафиксирована
					case 16:	a=500;	b=100;	break;							//	установка по горизонтали, все конечности опущены и свободны по горизонтали, кроме 6, которая поднята и зафиксирована
				}	if(iXP_time_LED>millis()){iXP_time_LED=0;}					//	произошло переполнение micros()
					if(a==0){digitalWrite(iXP_pins_LED, LOW );}else				//	выключаем светодиод
					if(b==0){digitalWrite(iXP_pins_LED, HIGH);}else				//	включаем светодиод
							{digitalWrite(iXP_pins_LED, iXP_time_LED+a>=millis()?HIGH:LOW);}//	мигаем светодиодом
					if(iXP_time_LED+a+b<millis()){iXP_time_LED=millis();}		//	обновляем iXP_time_LED
			}
		}