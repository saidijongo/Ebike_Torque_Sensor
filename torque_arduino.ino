/******************************************************************

//https://github.com/Chris741233/torque_to_throttle/blob/main/torque_to_throttle/torque_to_throttle.ino

Description : Arduino, test e-bike Torque/PAS to throttle 

*** Github (code, diagram, infos and more) :  https://github.com/Chris741233/torque_to_throttle

*** Forum Cyclurba : https://cyclurba.fr/forum/751958/arduino-l-assistance-d-un-vae.html?from=221&discussionID=31032&messageID=751958

- Torque Measurement and Control for Electric-Assisted Bike : https://www.mdpi.com/1424-8220/23/10/4657  (voir le Pdf)
- Divers : RC Low-pass Filter Design Tool http://sim.okawa-denshi.jp/en/CRtool.php


Torque sensor : AXLETORQUEBB68MM (Aliexpress)
5V 50mA MAX, 35mA standard
sortie 0.8-3.7v, 44mV/Nm, 75 Nm max

Consomation Arduino Nano (clone) en fonction (PAS et tork), env. 25mA sous 5.00V (alimentation labo precise sur pin +5V)

******************************************************************/


// -------- INCLUDE --------------

#include <RunningMedian.h>   // doc: https://github.com/RobTillaart/RunningMedian 

//#include <MemoryFree.h>      // test memoire dispo



// -------- PIN ARDUINO UNO/NANO --------------

// Pin indispensables
const int PIN_TORK = 1;      // A1 (ADC) entree capteur de couple 
const int PIN_PWM =  3;      // D3 (PWM) sortie throttle pour controleur
const int PIN_PAS_IN =  2;   // D2 pin Interrupt (!) : entree PAS, sans montage de resistance (cf INPUT_PULLUP) 

// branchements non obligatoires, constantes a laisser comme ca meme si vous n'en avez pas besoin
const int PIN_PAS_OUT = 4;   // sortie replication PAS, vers controleur : , cf const true/false plus bas
const int PIN_LED = 13;      // 13 = LED_BUILTIN, led interne Nano, pour controle turn_On ou turn_Off

const int PIN_DEBUG = 5;     // pour test uniquement, debug time loop oscillo (test loop ~1.75ms) 


// ---------------  CONSTANTS REGLABLES, A CALIBRER  --------
// ----------------------------------------------------------

// PAS
// ----------
const int  NB_MAGNETS =  32;  // How many magnets on TORQUE_PAS ? 

// Vref +5V Arduino
// --------------------
const float VREF = 5.00; // volt avec decimales : A relever precisement sur votre pin +5V, montage Arduino en fonction !

// Throttle : PWM 8bit = 5V/256 = 19.5mV precision (...)
// -------------------- pour les float = decimales obligatoires ! ------------------------------------------
const float FV_TR_SLEEP = 0.80;  // throttle au repos sans appui, volt avec decimales  
const float FV_TR_MIN = 1.00;    // throttle min,juste avant envoi assistance, volt avec decimales
const float FV_TR_MAX = 3.60;    // throttle max assistance (n'est pas forcement == a gachette poussee a fond !)


// Torque sensor : ADC 10bit 5V/1024 = ~4.88mV precision
// -------------- doc  0.8-3.7v, 44mV/Nm, 75 Nm max ---------------------------------
const float FV_TORK_MIN = 0.80;    // Tork sensor min, au repos, volt avec decimales
const float FV_TORK_MAX = 3.70;    // Tork sensor max signal, volt avec decimales


// Duplication PAS sur controleur (optionel), prend le relais si Tork est trop faible
// ---------------------------------------------
const bool PAS_CONTROLER = false; // Duplication PAS sur controleur ?  false par defaut
const bool PAS_32_8 = false;      // Diviser la sortie 32 aimants en 8 aimants ? false par defaut. Attention KEEP_NB_TURN doit etre == 1 !! 
// Torque mini: En dessous le PAS du controleur prend le relais (si true)
const float FV_OFFSET_PAS = 0.10;  // +volt avec decimales, est ajoute a FV_TORK_MIN, defaut +0.10V (doit être sup. a 0.00 !)


// Controle pedalage
// ------------------
const int START_NB_TURN = 1; // Aide au demarage : combien de tour pédalier ? Defaut 1 tour (min 1 !). En principe laisser sur 1.
const int PULSE_MIN = 2;     // controle nb. de pulse mini avant demarage (turn-on) defaut 3 : attention doit etre > 1 (min 2) 

// demarage optionel (non utilise)
//const int TORK_START = 300; // aide au demarage, valeur tork a envoyer, doit etre > tork_min et <= tork_max


// check PAS et Time to stop
// ---------------
const int CHECK_PAS = 500; // ms, time to stop, default 300ms, si l'assistance coupe trop vite à l'arrêt pedalage  mettre ~400-500ms 
const int DELAY_OFF = 50;  // ms, attente avant reprise si arret pedalage (min 0, defaut 50, max selon envie mais attention est bloquant)



// --- FILTRAGE SIGNAL TORK ------
// -------------------------------

// Les options sont appliques a la valeur high_tork dans l'ordre suivant :
// 1: KEEP_NB_TURN  // A laisser en principe sur 1 !
// 2: CUT_PIC_TORK  // en %
// 3: SAMPLE_MEDIAN  // filtre median
// 4: USE_TR_SPLINE // oui, non, offset ou pas

// ATTENTION AUX CUMULS !

const int KEEP_NB_TURN = 1;  // En principe laisser sur 1 - combien de tour pedalier a conserver avant mise a jour high_tork ? Defaut = 1 (min 1 !)

const int  CUT_PIC_TORK = 0; // % max augmentation tork par tour, sans decimales, expl: 25, 0=Off (desactive)

const int SAMPLE_MEDIAN = 0; // combien d'echantillon a placer dans le buffer filtre median ? min=3, 0=Off (desactive)
// expl: 3 = 3 tours pedalier pour remplir le buffer circulaire si KEEP_NB_TURN = 1 


// -- Throttle spline (verifier sotie dans setup debug ou fichier tableur "spline_throttle.xls")
#define USE_TR_SPLINE 0 // utiliser spline ? 1 true / 0 false (0 = filtre Off)  
#define SPLINE_OFFSET 1 // utiliser offset ? 1 true / 0 false (si offset alors depart courbe a U_MIN, else depart a 0)

const float U_MIN = FV_TR_MIN;   // debut courbe throttle, en principe == FV_TR_MIN
const float U_MAX = FV_TR_MAX;   // fin courbe throttle, en principe == FV_TR_MAX 
// Max courbure spline, float, decimale obligatoire 
const float COEFF_B = 2.5;    
// si offset:        B minimum = 2.5 obligatoire ! (voir tableur) 
// si pas d'offset:  B minimum = 1.0  (1.0 == pas de courbure du tout) 


// -- Debug Serial : Afficher les infos console serial ? true/false 
// remettre a false apres test, consomme des ressources et du temps si actif 
const bool USE_DEBUG = false;     



// --------- CONSTANTES CALCULEES, NE PAS MODIFIER A PARTIR D'ICI ----------
// --------------------------------------------------------------------------

// DOCU : ADC divise par 1023 ou 1024 ? cf doc  https://www.best-microcontroller-projects.com/arduino-adc.html 
// ADC expl:  1023 * (5/1024) = 4.9951V  (= Ok) ...  et non pas 1023 * (5/1023) = 5.0 (= faux, il exist un offset, cf doc !)

const int TR_SLEEP = round(FV_TR_SLEEP / VREF * 256); // conv PWM
const int TR_MIN = round(FV_TR_MIN / VREF * 256);     // conv PWM 
const int TR_MAX = round(FV_TR_MAX/VREF * 256);       // conv PWM 

const int TORK_MIN = round(FV_TORK_MIN/VREF * 1024);      // conv ADC  
const int TORK_MAX = round(FV_TORK_MAX/VREF * 1024);      // conv ADC  

const int OFFSET_PAS = round(FV_OFFSET_PAS/VREF * 1024);
const int TORK_MIN_BEFORE = TORK_MIN + OFFSET_PAS;  // valeur en ADC int

// calcul var throttle spline 
#if SPLINE_OFFSET
    const float Y_MAX = U_MAX - U_MIN; // expl : 3.60 - 1.00 = 2.60  
    const float COEFF_A = Y_MAX / pow(U_MAX, COEFF_B); 
#else
    const float Y_MAX = U_MAX; 
    const float COEFF_A = Y_MAX / pow(U_MAX, COEFF_B); 
#endif


// pour calcul rpm
const long COEFF_RPM = 60000 / NB_MAGNETS; // expl: 60000 / 6 magnets  = 10000  (1 minute=60000ms), ne pas modifier !


// ----------- VARIABLES GLOBALES, NE PAS MODIFIER --------------
// ---------------------------------------------------------------

// timer isr_pas
unsigned long isr_oldtime = 0;       

// var Interrupt isr_pas : volatile obligatoire si modif dans autres fonctions que isr
volatile unsigned int period_h = 0;  // period Pas high
volatile unsigned int period_l = 0;  // period Pas low 
volatile unsigned int period = 0;    // tot period
volatile unsigned int pulse = 0;     // PAS pulse count  (cf interrupt isr_pas)
volatile unsigned int pulse2 = 0;    // PAS pulse2 count (cf interrupt isr_pas)

// autres
unsigned int high_tork = 0;     // var. maximum tork sensor
unsigned int old_high_tork = 0; // pour comparaison augmentation tork
unsigned int pwm_out = 0;       // var. PWM out pour sortie Throttle controleur

unsigned int rpm  = 0;          // RPM pedalling
unsigned int tour = 0;          // nb de tour pedalier

bool ped_forward = false;       // pedaling forward or backward
bool led_state   = false;       // state led controle turn-on
bool pas_out     = false;           // etat pas_controler 


// -- creation objet filtre RunningMedian
RunningMedian samples = RunningMedian(SAMPLE_MEDIAN);



// ----------- MAIN ------------------
// -----------------------------------


void setup() {
    
    // -- Serial (for debug)
    Serial.begin(115200);
    
    pinMode(PIN_TORK, INPUT);
    pinMode(PIN_PWM, OUTPUT);
    
    pinMode(PIN_PAS_IN, INPUT_PULLUP); // INTERRUPT ! PAS Hall, without resistor (input_pullup) !
    
    pinMode(PIN_PAS_OUT, OUTPUT);      // replication PAS pour controler, si utilise
    pinMode(PIN_LED, OUTPUT);
    
    pinMode(PIN_DEBUG, OUTPUT);
    digitalWrite(PIN_DEBUG, LOW);
    
    // -- INTERRUPT pedaling : appel "isr_pas" sur signal CHANGE (high or low)
    attachInterrupt(digitalPinToInterrupt(PIN_PAS_IN), isr_pas, CHANGE);
    
    digitalWrite(PIN_LED, LOW);      // Led control, low in boot (high if turn On)
    digitalWrite(PIN_PAS_OUT, LOW);  
    
    Serial.print F("USE_DEBUG = "); Serial.println(USE_DEBUG);
    
    if(USE_DEBUG) {
        Serial.print F("PWM TR_SLEEP = "); Serial.println(TR_SLEEP);
        Serial.print F("PWM TR_MIN = "); Serial.println(TR_MIN);
        Serial.print F("PWM TR_MAX = "); Serial.println(TR_MAX);
        Serial.print F("ADC TORK_MIN = "); Serial.println(TORK_MIN);
        Serial.print F("ADC TORK_MAX = "); Serial.println(TORK_MAX);
        if(PAS_CONTROLER) {Serial.print F("ADC TORK_MIN_BEFORE = "); Serial.println(TORK_MIN_BEFORE);}
        Serial.println F("----------------"); 
        
        // test listing sortie courbe throttle
        #if USE_TR_SPLINE
            Serial.print F("COEFF_B = "); Serial.println(COEFF_B);
            Serial.print F("Start = "); Serial.println(U_MIN);
            Serial.print F("End = "); Serial.println(U_MAX);
            #if SPLINE_OFFSET
                for(float i=U_MIN; i<=U_MAX+0.1; i+=0.1)
                {
                    float x = float_spline(i);
                    Serial.print (i,3); Serial.print (";"); Serial.println(x,3);
                } 
            #else
                for(float i=0; i<=U_MAX+0.1; i+=0.1)
                {
                    float x = float_spline(i);
                    Serial.print (i,3); Serial.print (";"); Serial.println(x,3);
                } 
            #endif
            Serial.println F("----------------");  
        #endif
    } // endif debug
    
    Serial.println F("loop starting");
    turnOff();
    delay(100);   
    
} // endsetup



void loop() {
    
    digitalWrite(PIN_DEBUG, HIGH); // debug time loop oscillo
    
    static uint32_t oldtime1 = millis(); // timer1 (pour time to stop et divers) var type static !
    
    static unsigned int maxt;
    float pct, pct2;
    
    
    // ---- GESTION SIGNAL TORQUE IN  et CONTROLE PAS ---------
    // ----------------------------------------------------
    
    //unsigned int adc = analogRead(PIN_TORK); // read ADC tork sensor
    unsigned int adc = readADC(PIN_TORK, 10);  // moyenne de 10 echantillons de mesure ADC
    
    // -- control pedaling forward or backwards  (verif si pedalage en avant ou en arriere)
    if (period_h >= period_l) ped_forward = true;
    else ped_forward = false;
    
    
    // durant X tour pedalier on garde le tork le plus haut
    if (pulse2 > 0 && pulse2 <= NB_MAGNETS * KEEP_NB_TURN) 
    {
        if(adc < TORK_MIN) adc = TORK_MIN; // empeche assistance bloquee  
        if(adc > maxt) maxt = adc;         // garde la val.la plus haute provisoire
    }  
    
    // si = X tour ped on update var globale high_tork
    if(pulse2 >= NB_MAGNETS * KEEP_NB_TURN) {
        
        // -- limiter pic trop puissant
        // si augmentation depuis der. valeur
        if(CUT_PIC_TORK > 0) {
            if(maxt > old_high_tork) {
                pct = ((float(maxt) - float(old_high_tork))/float(old_high_tork))*100.0; // calcul % augmentation 
                
                // limiter a % max
                if(pct > float(CUT_PIC_TORK)) {
                    pct2      = (float(old_high_tork)/100.0)*float(CUT_PIC_TORK); 
                    high_tork = old_high_tork + pct2;
                    
                    //Serial.print(F("Free RAM = ")); Serial.println(freeMemory());  // how much RAM is available in bytes ?
                }
                else high_tork = maxt;
            }
            // si == ou diminution
            else {
                high_tork = maxt; 
            }
        } // endif CUT_PIC
        else {
            high_tork = maxt; // update high_tork si CUT_PIC == Off
        }
        
        // si filtre median actif
        if(SAMPLE_MEDIAN > 0) { 
            samples.add(high_tork);             // ajout echantillon  au filtre median
            high_tork = samples.getMedian();    // update high_tork with sample median
            //high_tork = samples.getAverage(); // update high_tork with sample average (choix moyenne ou median !)
        }
        
        
        old_high_tork = high_tork; // update old value high_tork
        maxt          = 0;         // raz high provisoire
        pulse2        = 0;         // raz  pulse2 (cf isr)
        tour += KEEP_NB_TURN;      // incremente nb. de tour pedalier depuis demarage
        
    } // endif update high_tork
    
    
    ////// Tork  moyen pour le demarage du (ou des) premier tour = Aide au demarage (desactive)
    ////// if(tour < NB_TURN && pulse >= PULSE_MIN && high_tork < TORK_START) 
    
    // Demarage : garde la val.la plus haute du 1er tour en direct (ou plus selon constante)
    if(tour < START_NB_TURN) {
        //high_tork = TORK_START; // si valeur aide fixe
        if(adc > high_tork) high_tork = adc; 
        old_high_tork = high_tork;
    }
    
    
    // ----  SIGNAL THROTTLE OUT (pwm_out) -------
    // -------------------------------------------
    
    // -- si  PAS_CONTROLER == true
    if (PAS_CONTROLER) {
        if( high_tork < TORK_MIN_BEFORE) {
            if(tour > START_NB_TURN) {
                pwm_out = TR_SLEEP; // si true, throttle min/stop == PAS controler prend le relais !
                // le throttle ne doit pas envoyer d'assistance ! Le cas echeant verifier valeur TR_MIN 
                pas_out = true; // = replication signal PAS
            }
        }
        else {
            pas_out = false;
            digitalWrite(PIN_PAS_OUT, LOW);
            
            //  MAP sortie PWM throttle --
            map_pwm();   
        }
    }
    else {
        // -- si  PAS_CONTROLER == false
        pas_out = false;
        
        // MAP sortie PWM throttle -- 
        map_pwm();
        
    } //end
    
    
    // ---- CONTROLE ASSISTANCE ON-OFF ---------   
    // --------------------------------
    
    // -- Turn On only if (assistance seulement si) ... 
    if(tour < START_NB_TURN) {
        if (high_tork > TORK_MIN || pulse >= PULSE_MIN ) turnOn();  // ||=OR, pour 1er tour (ou plus selon constante)
    }
    else {
        if (high_tork >= TORK_MIN && pulse >= PULSE_MIN ) turnOn(); // &&=AND, pour les tours d'apres
    } 
    
    
    // ---- TIMER1 CONTROLE ---------------------------------- 
    // check RPM, Turn Off if no pedalling, debug serial if true 
    // --------------------------------------------------------
    
    uint32_t check_t1 = millis() - oldtime1;
    
    if (check_t1 >= CHECK_PAS) {
        
        oldtime1 = millis();   // update timer loop
        
        rpm = (pulse * COEFF_RPM) / check_t1; // calcul rpm pour info (utiliser pulse pour les controles prog. !)
        
        // -- turn off if no pedaling ...
        if (pulse == 0 || period == 0)  turnOff();
        
        // -- turn off if ped. backward (pedalage en arriere !)
        if (ped_forward==false) turnOff(); 
        
        
        // -- Debug serial
        //float volt = pwm_out;
        if (USE_DEBUG) {
            
            
            //Serial.print(F("Free RAM = ")); Serial.println(freeMemory());  // how much RAM is available in bytes ?
            if (led_state==0) Serial.println F("Turn OFF !");
            
            Serial.print F("Rpm : "); Serial.println(rpm);
            Serial.print F("tour : "); Serial.println(tour);
            Serial.print F("pulse : "); Serial.println(pulse);
            Serial.print F("pulse2 : "); Serial.println(pulse2);
            Serial.print F("period : "); Serial.println(period);
            Serial.print F("state : "); Serial.println(led_state);
            Serial.print F("high_tork : "); Serial.println(high_tork); 
            Serial.print F("pwm_out : "); Serial.println(pwm_out);
            //Serial.print F("pwm V= "); Serial.println((volt*VREF)/255);
            
            if(PAS_CONTROLER && pas_out && led_state) Serial.print F("PAS en fonction !"); 
            Serial.println F("-----------");
            
            
            // datalog CSV
            //Serial.print(rpm); Serial.print(";");
            //Serial.print(tour); Serial.print(";");
            //Serial.print(led_state); Serial.print(";");
            //Serial.print(pct); Serial.print(";");
            //Serial.print(pct2); Serial.print(";");
            //Serial.print(high_tork); Serial.print(";");
            //Serial.println(pwm_out); 
        }
        
        detachInterrupt(digitalPinToInterrupt(PIN_PAS_IN)); // (recommended)
        // https://www.arduino.cc/reference/en/language/functions/external-interrupts/detachinterrupt/
        
        pulse = 0;         // reset pulses PAS
        
        attachInterrupt(digitalPinToInterrupt(PIN_PAS_IN), isr_pas, CHANGE);
        
    } 
    // ------ endif timer 1 --------------
    // -----------------------------------
    
    
    // LED status info (turn-On or turn-Off)
    digitalWrite(PIN_LED, led_state);
    
    // -- dont put delay() here !
    
    digitalWrite(PIN_DEBUG, LOW); // debug time loop oscillo  
} // endloop




// -------- FUNCTIONS ----------------------
// -----------------------------------------

// -- traitement sortie throttle
void map_pwm() {
 
    // butee plage tork
    high_tork = constrain(high_tork, TORK_MIN, TORK_MAX);  // limits range of values
    
    #if USE_TR_SPLINE
        int temp_pwm = map(high_tork, TORK_MIN, TORK_MAX, TR_MIN, TR_MAX); // re-map value (0-1023 to 0-255)
        pwm_out      = int_spline(temp_pwm); // applique spline throttle, cf constantes  
    #else
        pwm_out = map(high_tork, TORK_MIN, TORK_MAX, TR_MIN, TR_MAX); // re-map value (0-1023 to 0-255)
    #endif
    
    // butee plage throttle
    pwm_out = constrain(pwm_out, TR_SLEEP, TR_MAX);  // limits range of values
    
} // endfunc



// -- Turn on output and set state variable to true
void turnOn() {
    
    analogWrite(PIN_PWM, pwm_out); // write PWM throttle 
    led_state = true;
    
} // endfunc


// -- Turn off output, reset pulse counter and set state variable to false
void turnOff() {
    
    pulse  = 0;
    pulse2 = 0;
    
    tour          = 0;
    high_tork     = TORK_MIN;
    old_high_tork = high_tork;
    
    pwm_out = TR_SLEEP;           // anciennement TR_MIN
    
    analogWrite(PIN_PWM, pwm_out);  // PWM throttle minimum 
    led_state = false;
    
    if(SAMPLE_MEDIAN > 0) samples.clear();   // raz buffer median
    
    delay(DELAY_OFF);  // delay, duree du Off, cf reglages constantes
    
} // endfunc


// -- ISR - INTERRUPT PAS
void isr_pas() {
    
    // rester ici le plus concis possible
    uint32_t isr_time = millis();
    
    if (digitalRead(PIN_PAS_IN) == HIGH) {
        period_l = (isr_time - isr_oldtime);  // ms low (inverse)
        
        // -- replication PAS pour controleur (si true et en fonction)
        if(PAS_CONTROLER && pas_out) { 
            
            // option reduire signal 32 aimants en 8, low
            if(NB_MAGNETS==32 && PAS_32_8 && KEEP_NB_TURN==1) {
                if(pulse2 >= 0 && pulse2 < 2) digitalWrite(PIN_PAS_OUT, LOW); 
                if(pulse2 >= 4 && pulse2 < 6) digitalWrite(PIN_PAS_OUT, LOW);
                if(pulse2 >= 8 && pulse2 < 10) digitalWrite(PIN_PAS_OUT, LOW);
                if(pulse2 >= 12 && pulse2 < 14) digitalWrite(PIN_PAS_OUT, LOW);
                if(pulse2 >= 16 && pulse2 < 18) digitalWrite(PIN_PAS_OUT, LOW); 
                if(pulse2 >= 20 && pulse2 < 22) digitalWrite(PIN_PAS_OUT, LOW);
                if(pulse2 >= 24 && pulse2 < 26) digitalWrite(PIN_PAS_OUT, LOW);
                if(pulse2 >= 28 && pulse2 < 30) digitalWrite(PIN_PAS_OUT, LOW);
            }
            else {
                digitalWrite(PIN_PAS_OUT, HIGH); 
            }
        } //endif PAS_CONTROLER low
        
        pulse++;   // increment nb de pulse (pour calcul rpm et controles turn-on)
        pulse2++;  // increment pulse2 (divers)
    }
    else {
        period_h = (isr_time - isr_oldtime);  // ms high (inverse)
        
        // replication PAS pour controleur (si true et en fonction)
        if(PAS_CONTROLER && pas_out) {
            
            // option reduire signal 32 aimants en 8, high
            if(NB_MAGNETS==32 && PAS_32_8 && KEEP_NB_TURN==1) {
                if(pulse2 >= 2 && pulse2 < 4) digitalWrite(PIN_PAS_OUT, HIGH); 
                if(pulse2 >= 6 && pulse2 < 8) digitalWrite(PIN_PAS_OUT, HIGH);
                if(pulse2 >= 10 && pulse2 < 12) digitalWrite(PIN_PAS_OUT, HIGH);
                if(pulse2 >= 14 && pulse2 < 16) digitalWrite(PIN_PAS_OUT, HIGH);
                if(pulse2 >= 18 && pulse2 < 20) digitalWrite(PIN_PAS_OUT, HIGH); 
                if(pulse2 >= 22 && pulse2 < 24) digitalWrite(PIN_PAS_OUT, HIGH);
                if(pulse2 >= 26 && pulse2 < 28) digitalWrite(PIN_PAS_OUT, HIGH);
                if(pulse2 >= 30 && pulse2 < 32) digitalWrite(PIN_PAS_OUT, HIGH);
            }
            else {
                digitalWrite(PIN_PAS_OUT, LOW); 
            }
        } //endif PAS_CONTROLER high
    }
    
    period = period_h + period_l;   // tot period
    
    isr_oldtime = isr_time;        // update timer interupt
    
} // endfunc isr


// -- moyennage ADC
int readADC(int adc, int n) { //take and average n values to reduce noise; return average
    int reading = analogRead(adc); //dummy read
    reading     = 0;
    for (int i = 0; i < n; i++) {
        reading += analogRead(adc);
    }
    float average   = reading / n;
    int average_int = round(average);  // float arrondi
    return (average_int);
    
} // endfunc


// -- formule avec ou sans offset, sortie en integer pour PWM 
int int_spline(int val) {
    
    float f_val = (val * VREF) / 256; // convertir val en float
    
    float x = COEFF_A * (pow(f_val, COEFF_B)); // formule
    
    #if SPLINE_OFFSET
        x += U_MIN; // ajout offset au resultat
        if(x < U_MIN + 0.06) x = U_MIN;  // correction premieres valeurs (formule avec offset a ameliorer !)
    #endif
    
    int y = round(x / VREF * 256);   // conv. integer pour PWM
    
    //Serial.print(F("Free RAM = ")); Serial.println(freeMemory());  // how much RAM is available in bytes ?
    
    return y;
    
} // endfunc


// -- formule pour debug serial uniquement (verif courbe, sortie en float)
float float_spline(float val) {
    
    float x = COEFF_A * (pow(val,COEFF_B)); // formule
    
    #if SPLINE_OFFSET
        x += U_MIN; // ajout offset au resultat
        if(x < U_MIN + 0.06) x = U_MIN;  // correction premiere valeur (formule avec offset a ameliorer !)
    #endif
    
    return x;
    
} // endfunc
