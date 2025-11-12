/*
    usb.c - Gestione interfaccia USB CDC-ACM (porta seriale virtuale)
    Autore: Ivan Giampieri
    Data: 2025-10-03
    Descrizione:
    Questo file fornisce un canale di comunicazione seriale USB interattivo, 
    usato per configurare, diagnosticare e controllare le funzioni BLE del sistema EMGyro2.
    
    Implementa la gestione completa della comunicazione USB tramite la classe CDC-ACM,
    che consente al dispositivo di apparire come una porta seriale virtuale (Virtual COM Port) al PC.
    Oltre alla semplice trasmissione e ricezione di dati, il modulo include:
      - L’inizializzazione dello stack USB Nordic e della classe CDC-ACM;
      - La gestione degli eventi USB;
      - La ricezione e l’interpretazione di comandi testuali inviati dal terminale USB;
      - L’interfaccia con il modulo Bluetooth per l’esecuzione di comandi;
      - La possibilità di eseguire un reset software del dispositivo tramite comando USB.
*/

#include "usb.h"

#include <stddef.h>
#include <stdio.h>
#include "nrf_delay.h"

#include "app_usbd_core.h"        // Core USB stack Nordic
#include "app_usbd.h"             // Gestione generale USBD
#include "app_usbd_string_desc.h" // Stringhe USB (descrittori)
#include "app_usbd_cdc_acm.h"     // Classe CDC-ACM (seriale virtuale)
#include "app_usbd_serial_num.h"  // Generazione numero seriale USB

#include "bluetooth.h"   // per ble_forget_bonds()

#include "nrf_log.h"       // Include l'API principale del modulo di logging di Nordic (NRF_LOG).
                        // Contiene le macro e le funzioni come NRF_LOG_INFO, NRF_LOG_ERROR, ecc.,
                        // che permettono di stampare messaggi di debug su UART o RTT.

#include "nrf_log_ctrl.h"  // Include funzioni di controllo avanzato del logging.
                        // Necessario per chiamare NRF_LOG_FLUSH(), che forza la scrittura
                        // immediata di tutti i messaggi pendenti nei backend (UART/RTT).
                        // Senza questo include, NRF_LOG_FLUSH non è definita e il linker
                        // genererebbe errore.

// Sequenze ANSI per colorare testo in terminale
#define ANSI_RESET      "\x1b[0m"
#define ANSI_RED        "\x1b[31m"
#define ANSI_GREEN      "\x1b[32m"
#define ANSI_YELLOW     "\x1b[33m"
#define ANSI_BLUE       "\x1b[34m"
#define ANSI_MAGENTA    "\x1b[35m"
#define ANSI_CYAN       "\x1b[36m"
#define ANSI_WHITE      "\x1b[37m"

/*------------------- INCLUSIONE FILE DI INTESTAZIONE ------------------------------------------
    Questa prima sezione include tutti i file di intestazione necessari per il funzionamento del modulo USB.
    - I file `app_usbd_*` contengono le librerie Nordic per gestire la comunicazione USB a basso livello.
    - I file `nrf_log` e `nrf_log_ctrl` permettono di gestire messaggi di log e debug.
    - Infine, `bluetooth.h` è incluso per poter richiamare funzioni che interagiscono con il BLE, 
      come la cancellazione o la gestione dei bond.
*///--------------------------------------------------------------------------------------------

/*------------------- CONFIGURAZIONE CDC-ACM ------------------------------------------------------------
    Questa sezione definisce i parametri di configurazione della classe USB CDC-ACM.
    La classe CDC-ACM permette di creare una "seriale virtuale" via USB, cioè una porta COM visibile al PC.
    Qui vengono configurati gli endpoint di comunicazione (canali logici di input/output) e
    i buffer utilizzati per la trasmissione e la ricezione dei dati.
*///-----------------------------------------------------------------------------------------------------

// Interfacce ed endpoint USB per CDC-ACM
#define CDC_ACM_COMM_INTERFACE  0  // Interfaccia per la comunicazione (comandi)
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2 // Endpoint IN per comunicazione
#define CDC_ACM_DATA_INTERFACE  1  // Interfaccia per i dati
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1 // Endpoint IN per dati
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1 // Endpoint OUT per dati

#define READ_SIZE 1   // leggo 1 byte alla volta (eco immediato)

// Buffer di ricezione e trasmissione
char m_rx_buffer[READ_SIZE];  // buffer di 1 byte per lettura immediata
char m_tx_buffer[NRF_DRV_USBD_EPSIZE]; // buffer per invio dati (dimensione endpoint)

// Flag che controlla l’invio periodico
bool m_send_flag = false;   // <<< MODIFICATO (prima non usato)
// serve come segnalatore interno per gestire invii non continui ma condizionati


#define RX_BUFFER_SIZE 64
// Definisce la dimensione del buffer di ricezione. Qui possiamo memorizzare fino a 64 caratteri.

static char input_buffer[RX_BUFFER_SIZE];
// Buffer statico dove accumuliamo i caratteri ricevuti fino a formare un comando completo.

static size_t input_index = 0;
// Indice corrente nel buffer: indica la posizione in cui inserire il prossimo carattere.

/*----------------------------------------------------------------------------------
    Le variabili globali sopra sono usate per gestire il flusso dei dati USB.
    `m_rx_buffer` riceve byte in arrivo, uno alla volta.
    `input_buffer` accumula più caratteri per costruire un comando (ad esempio “help” o “lista”).
    `input_index` tiene traccia di dove scrivere il prossimo carattere ricevuto.
*///---------------------------------------------------------------------------------


/*----------------------------------------------------------------------------------------------
    Questa funzione è il cuore della gestione della porta seriale virtuale USB.
    È un *event handler* chiamato automaticamente dallo stack Nordic ogni volta che si verifica
    un evento USB importante (es. apertura porta, ricezione dati, chiusura, ecc.).
    A seconda del tipo di evento, il codice reagisce in modo diverso.
*/
// ------------- HANDLER EVENTI CDC-ACM --------------------------------------------------------
void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,app_usbd_cdc_acm_user_event_t event)
// Funzione di gestione degli eventi dell'interfaccia CDC ACM (USB virtual COM port).
// p_inst: puntatore all'istanza della classe USB.
// event: tipo di evento generato dall'interfaccia USB.
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);
    // Ottiene un puntatore alla struttura specifica della classe CDC ACM
    // a partire dall'istanza generica passata come parametro.

    switch (event)
    // Gestione degli eventi tramite switch-case
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        // Evento: la porta USB è stata aperta dal PC
        {
            ret_code_t ret = app_usbd_cdc_acm_read(p_cdc_acm, m_rx_buffer, READ_SIZE);
            // Avvia la prima lettura asincrona dalla porta USB.
            // m_rx_buffer è il buffer temporaneo dove verranno memorizzati i dati letti.
            // READ_SIZE indica quanti byte leggere.

            UNUSED_VARIABLE(ret);
            // Macro per evitare warning di compilazione se il valore di ritorno non viene usato.
            
            m_send_flag = true;                 // Attiva flag di porta aperta
            // Messaggio di benvenuto
            const char *benvenuto = "\r\n"
                                    "         [ EMGyro2 ]\r\n"
                                    "----------------------------------------\r\n"
                                    "Sistema avviato correttamente.\r\n"
                                    "Benvenuto! Porta USB aperta.\r\n"
                                    "----------------------------------------\r\n"
                                    "Scrivi 'help' per visualizzare i comandi disponibili.\r\n"
                                    "Come posso aiutarla oggi?\r\n";

            usb_send(benvenuto, strlen(benvenuto)); // invia il messaggio di benvenuto al PC
            m_send_flag = false; 
            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        // Evento: dati ricevuti tramite USB
        {
            size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
            // Ottiene quanti byte sono stati ricevuti.

            char c = m_rx_buffer[0];  // carattere ricevuto
            // Prende il primo carattere ricevuto dal buffer di ricezione temporaneo.

            if (c == '\r' || c == '\n') {
                // Controlla se il carattere indica fine riga (invio)
                input_buffer[input_index] = '\0'; // termina stringa
                // Aggiunge il terminatore null per trasformare il buffer in una stringa C valida

                // In questo punto il codice interpreta la stringa ricevuta come un comando.
                // Se ad esempio l'utente ha digitato "help", viene riconosciuto qui.
 
                // Esegui il comando
                if (strcmp(input_buffer, "help") == 0) {
                    usb_send("\r\n"ANSI_GREEN " > info\r\n > list\r\n > bond_info\r\n > clear <id>\r\n > forget\r\n > reset\r\n\r\n"ANSI_RESET"\r\n",
                         strlen("\r\n"ANSI_GREEN"> info\r\n > list\r\n > bond_info\r\n > clear <id>\r\n > forget\r\n > reset\r\n\r\n"ANSI_RESET"\r\n"));
                
                } else if (strcmp(input_buffer, "info") == 0) {
                    usb_send("\r\n"ANSI_CYAN"info:\r\n"
                            " > list        - Mostra i dispositivi Bluetooth memorizzati o connessi\r\n"
                            " > bond_info   - Mostra i dettagli delle connessioni Bluetooth abbinate\r\n"
                            " > clear <id>  - Cancella un dispositivo specifico dalla memoria\r\n"
                            " > forget      - Elimina tutte le associazioni Bluetooth salvate\r\n"
                            " > reset       - Riavvia la scheda EMGyro2\r\n\r\n"ANSI_RESET "\r\n",
                    strlen("\r\n"ANSI_CYAN"info:\r\n"
                            " > list        - Mostra i dispositivi Bluetooth memorizzati o connessi\r\n"
                            " > bond_info   - Mostra i dettagli delle connessioni Bluetooth abbinate\r\n"
                            " > clear <id>  - Cancella un dispositivo specifico dalla memoria\r\n"
                            " > forget      - Elimina tutte le associazioni Bluetooth salvate\r\n"
                            " > reset       - Riavvia la scheda EMGyro2\r\n\r\n"ANSI_RESET "\r\n"));
        
                } else if (strcmp(input_buffer, "forget") == 0) {
                    usb_send("\r\n"ANSI_YELLOW"Cancellazione bond BLE in corso...\r\n"ANSI_RESET"\r\n", 
                        strlen("\r\n"ANSI_YELLOW"Cancellazione bond BLE in corso...\r\n"ANSI_RESET"\r\n"));
                    ble_forget_bonds();  // Richiama la funzione BLE nel bluetooth.c
                    usb_send(ANSI_GREEN"FATTO!\r\n"ANSI_RESET"\r\n", strlen(ANSI_GREEN"FATTO!\r\n"ANSI_RESET"\r\n"));
                    
                } else if (strcmp(input_buffer, "list") == 0) {
                    usb_send("\r\n"ANSI_BLUE"Ecco la lista:\r\n"ANSI_RESET, 
                        strlen("\r\n"ANSI_BLUE"Ecco la lista:\r\n"ANSI_RESET));
                    ble_list_bonds();   // Richiama la funzione BLE nel bluetooth.c
        
                } else if (strcmp(input_buffer, "bond_info") == 0) {
                    usb_send("\r\n"ANSI_MAGENTA"Mostro info sui bond...\r\n"ANSI_RESET"\r\n", 
                        strlen("\r\n"ANSI_MAGENTA"Mostro info sui bond...\r\n"ANSI_RESET"\r\n"));
                    ble_bond_info();
                    usb_send("\r\n", 2);

                } else if (strncmp(input_buffer, "clear", 5) == 0) {
                    int id;          // Controllo sintassi: deve esserci un numero dopo "clear"
                    if (sscanf(input_buffer, "clear %d", &id) == 1) {
                        char msg[64];
                        snprintf(msg, sizeof(msg), "\r\n"ANSI_YELLOW"Cancellazione bond %d in corso...\r\n"ANSI_RESET"\r\n", id);
                        usb_send(msg, strlen(msg));
                        ble_clear_bond((uint8_t)id);
                    } else {
                        usb_send("\r\n"ANSI_RED"Uso corretto: clear <id>\r\n"ANSI_RESET"\r\n", strlen("\r\n"ANSI_RED"Uso corretto: clear <id>\r\n"ANSI_RESET"\r\n"));
                    }

                } else if (strcmp(input_buffer, "reset") == 0) {
                    usb_send("\r\n"ANSI_MAGENTA"Reset in corso...\r\n"ANSI_RESET"\r\n", strlen("\r\n"ANSI_MAGENTA"Reset in corso...\r\n"ANSI_RESET"\r\n"));
                    NRF_LOG_FLUSH();          // Assicura che tutti i log vengano stampati
                    nrf_delay_ms(20);         // <-- Attendi 20 ms
                    NVIC_SystemReset();       // Esegue un soft reset della MCU
                
                } else {
                    usb_send("\r\n"ANSI_RED"Comando sconosciuto\r\n"ANSI_RESET, strlen("\r\n"ANSI_RED"Comando sconosciuto\r\n"ANSI_RESET));
                    // Comando non riconosciuto -> invia messaggio di errore
                }

                // Reset buffer
                input_index = 0;    // Resetta l'indice del buffer per il prossimo comando

                } else if (c == '\b' || c == 127) {
                        // Backspace o cancella carattere precedente
                        if (input_index > 0) {
                            input_index--;
                            usb_send("\b \b", 3);  // sposta indietro, cancella e torna indietro
                        }

                } else {
                    // Accumulo carattere nel buffer
                    if (input_index < RX_BUFFER_SIZE - 1) {
                        // Se non abbiamo superato la dimensione massima, aggiungiamo il carattere
                        input_buffer[input_index++] = c;   
                        usb_send(&c, 1); // eco sul terminale
                    }
                }

            // Riprendo lettura
            ret_code_t ret = app_usbd_cdc_acm_read(p_cdc_acm, m_rx_buffer, READ_SIZE);
            // Riavvia la lettura per continuare a ricevere dati USB
            UNUSED_VARIABLE(ret);
            // Ignora il valore di ritorno per evitare warning
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            // Evento: la porta USB è stata chiusa dal PC
            break;

        default:
            // Qualsiasi altro evento non gestito
            break;
    }
}

/*--------------------------------------------------------------------------------
    Questa parte crea un’istanza della classe CDC-ACM.
    In pratica “registra” la porta seriale virtuale USB nel sistema, specificando
    quali interfacce ed endpoint usare, e quale funzione gestirà gli eventi (handler).
*/
//------------------ Chiamo l'istanza CDC-ACM ------------------------------------
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,    // handler eventi
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

/*-------------------------------------------------------------------------------
    Qui viene definito l’handler per gli eventi generali USB, cioè quelli legati al
    collegamento fisico (inserimento o rimozione del cavo, sospensione, ecc.).
    Serve per gestire correttamente l’avvio e l’arresto dello stack USB.
*/
// ----------------------- GESTIONE USB -----------------------------------------
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            // USB sospesa (es. PC in standby)
            break;
        case APP_USBD_EVT_DRV_RESUME:
            // USB ripresa dopo sospensione
            break;
        case APP_USBD_EVT_STARTED:
            // Stack USB avviato correttamente
            break;
        case APP_USBD_EVT_STOPPED:
            // USB fermata → disabilito lo stack
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            // USB collegata: abilito stack se non già attivo
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            // USB scollegata: fermo lo stack
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            // USB pronta all’uso → avvio stack
            app_usbd_start();
            break;
        default:
            break;
    }
}

/*-----------------------------------------------------------------------------------
    Queste sono le funzioni pubbliche del modulo, cioè quelle che altri file possono chiamare.
    - `usb_init()` inizializza lo stack USB e la classe CDC-ACM.
    - `usb_start()` abilita la gestione degli eventi di alimentazione USB.
    - `usb_send()` invia dati sulla seriale virtuale.
    Sono funzioni tipiche per separare l’inizializzazione dall’esecuzione effettiva.
*/
// --------------------- FUNZIONI PUBBLICHE -----------------------------------------
int usb_init (void)
{
    ret_code_t ret;
    // Configurazione stack USB con handler eventi
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
 
    // Genera un numero seriale USB univoco dal chip
    app_usbd_serial_num_generate();
 
    // Inizializza stack USB
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
 
    // Aggiunge la classe CDC-ACM allo stack
    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);
 
    return ret;
}
 
int usb_start (void)
{
    ret_code_t ret;
    // Abilita la gestione eventi di alimentazione USB
    ret = app_usbd_power_events_enable();
    APP_ERROR_CHECK(ret);
    return ret;
}
 
int usb_send (const char *data, size_t size)
{
        int ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, data, size);
        return ret;
}