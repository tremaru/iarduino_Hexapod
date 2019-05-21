// Библиотека EEPROM для работы arduino с энергонезависимой памятью EEPROM
// Библиотека iarduino_MultiServo для работы arduino с Servo Shield на 16 каналов http://iarduino.ru/shop/Expansion-payments/servo-shield-na-16-kanalov.html
// Библиотека iarduino_Hexapod для работы arduino с Hexapod http://iarduino.ru/shop/Mehanika/robot-hexapod-shestinogiy.html
// Библиотека iarduino_IR для работы arduino с ИК-приёмопередатчиками
// Приём данных с ИК-пульта осуществляется через Trema ИК-приёмник http://iarduino.ru/shop/Sensory-Datchiki/ik-priemnik-trema-modul.html
// Управление осуществляется с помощью ИК-пульта http://iarduino.ru/shop/Sensory-Datchiki/infrakrasnyy-pult-distancionnogo-upravleniya-priemnik-hx1838.html
// Для удобства подключения предлагаем воспользоваться Trema Shield http://iarduino.ru/shop/Expansion-payments/trema-shield.html

// Подключаем библиотеки:
#include <EEPROM.h>                     //  Подключаем библиотеку для работы с EEPROM
#include <iarduino_MultiServo.h>        //  Подключаем библиотеку для работы с Multi Servo Shield
#include <iarduino_Hexapod.h>           //  Подключаем библиотеку для работы с HEXAPOD
#include <iarduino_IR.h>                //  Подключаем библиотеку для работы с ИК приёмником

// Объявляем переменные и константы:
iarduino_Hexapod IXP(2,3,4);            //  Объявляем переменную IXP, для работы с HEXAPOD       (номера цифровых выводов к которым подключены: кнопка A, кнопка B, светодиод)
iarduino_IR      IR (7);                //  Объявляем переменную IR,  для работы с ИК приёмником (номер  цифрового вывода к которому подключён ИК приёмник)
int              command  = COM_EMPTY;  //  Номер команды
int              parametr = COM_EMPTY;  //  Значение параметра команды
int              height   = 0;          //  Высота HEXAPOD, от -10 до 10
unsigned long    count_ms = 0;          //  Время поступления последней команды

void setup(){
   IXP.begin(SERVO_SG90);               //  Инициируем HEXAPOD
   IR.begin();                          //  Инициируем ИК приёмник
// IXP.calibration_reset();             //  Удаление ранее сохранённых параметров калибровки
}

void loop(){
//ПРИНИМАЕМ КОМАНДЫ
  if(IR.check(true)){   // если принят пакет с пульта (включая пакеты повторов)
    count_ms=millis();  // фиксируем время получения пакета
    switch(IR.data){    // проверяем какая кнопка пульта нажата
    /* CH-  = спать     */ case 0xFFA25D: command=COM_SLEEP;     parametr=COM_EMPTY;                break;
    /* CH   = рост +    */ case 0xFF629D: height++; if(height> 10){height= 10;} if(height>0){command=COM_UP; parametr=height;}else{command=COM_LAY; parametr=-height;} delay(100); break;
    /* CH+  = рост -    */ case 0xFFE21D: height--; if(height<-10){height=-10;} if(height>0){command=COM_UP; parametr=height;}else{command=COM_LAY; parametr=-height;} delay(100); break;
    /* <<   = идти ^ <  */ case 0xFF22DD: command=GO_ON_LEFT;    parametr=COM_EMPTY;                break;
    /* >>   = идти ^    */ case 0xFF02FD: command=GO_ON;         parametr=COM_EMPTY;                break;
    /* >||  = идти ^ >  */ case 0xFFC23D: command=GO_ON_RIGHT;   parametr=COM_EMPTY;                break;
    /* -    = идти   <  */ case 0xFFE01F: command=GO_LEFT;       parametr=COM_EMPTY;                break;
    /* +    = стоп      */ case 0xFFA857: command=COM_STOP;      parametr=COM_EMPTY;                break;
    /* EQ   = идти   >  */ case 0xFF906F: command=GO_RIGHT;      parametr=COM_EMPTY;                break;
    /* 0    = идти V <  */ case 0xFF6897: command=GO_BACK_LEFT;  parametr=COM_EMPTY;                break;
    /* 100+ = идти V    */ case 0xFF9867: command=GO_BACK;       parametr=COM_EMPTY;                break;
    /* 200+ = идти V >  */ case 0xFFB04F: command=GO_BACK_RIGHT; parametr=COM_EMPTY;                break;
    /* 1    = тест      */ case 0xFF30CF: command=COM_TEST;      parametr=COM_EMPTY;                break;
    /* 2    = встать    */ case 0xFF18E7: command=COM_UP;        parametr=10;        height= 10;    break;
    /* 3    = лежать    */ case 0xFF7A85: command=COM_LAY;       parametr=10;        height=-10;    break;
    /* 4    = сидеть    */ case 0xFF10EF: command=COM_SIT;       parametr=COM_EMPTY;                break;
    /* 5    = поклон    */ case 0xFF38C7: command=COM_NOD;       parametr=COM_EMPTY;                break;
    /* 6    = плав.брас */ case 0xFF5AA5: command=COM_EMPTY;     parametr=COM_EMPTY; IXP.walking=0; break;
    /* 7    = походка 1 */ case 0xFF42BD: command=COM_EMPTY;     parametr=COM_EMPTY; IXP.walking=1; break;
    /* 8    = походка 2 */ case 0xFF4AB5: command=COM_EMPTY;     parametr=COM_EMPTY; IXP.walking=2; break;
    /* 9    = походка 3 */ case 0xFF52AD: command=COM_EMPTY;     parametr=COM_EMPTY; IXP.walking=3; break;
    }
  }
//ВЫПОЛНЯЕМ КОМАНДЫ
  IXP.go(command,parametr);                                         // выполняем движение
//ОТМЕНЯЕМ КОМАНДЫ
  if(count_ms>millis()){count_ms=0;}                                // произошло переполнение millis()
  if(count_ms+250<millis()){command=COM_EMPTY; parametr=COM_EMPTY;} // стираем предыдущую команду, через 250мс после её поступления (останавливаем HEXAPOD, если не нажаты кнопки пульта)
//КАЛИБРОВКА
  IXP.calibration();                                                // калибровка суставов конечностей
}
