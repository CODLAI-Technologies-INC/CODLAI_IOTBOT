/*
 * 📌 Firebase özelliklerini kullanabilmek için, IOTBOT_Config.h dosyasında 'USE_FIREBASE' tanımının başındaki
 * yorum satırlarını (//) kaldırın. Aksi takdirde, Firebase işlevleri devre dışı kalacaktır.
 *
 * 📌 To enable Firebase features, remove the comment (//) before the 'USE_FIREBASE' definition in
 * IOTBOT_Config.h. Otherwise, Firebase functions will be disabled.
 *
 * Firebase yapılandırma adımları / Firebase setup steps:
 * 1. Firebase Console > Realtime Database bölümünde veritabanı oluşturun ve URL'yi kopyalayın ("https://...firebaseio.com/").
 *    / Create a database under Firebase Console → Realtime Database and copy its URL.
 * 2. Project Settings > General sekmesinde "Web API Key" alanını bulun ve `FIREBASE_API_KEY` değerine yazın.
 *    / In Project Settings → General, copy the Web API Key into `FIREBASE_API_KEY`.
 * 3. Authentication > Sign-in method menüsünde Email/Password girişini etkinleştirin ve "Users" sekmesinden bir e-posta hesabı
 *    oluşturun. / Enable Email/Password in Authentication → Sign-in method and create a user under the Users tab; place the
 *    credentials into `USER_EMAIL` and `USER_PASSWORD`.
 * 4. `USE_FIREBASE` tanımlandığında WiFi yardımcıları otomatik olarak açılır; koda ekstra `#define USE_WIFI` eklemeniz gerekmez.
 *    / Defining `USE_FIREBASE` automatically pulls in the Wi-Fi helpers, so no extra `#define USE_WIFI` is required.
 */
#define USE_FIREBASE
#include <IOTBOT.h>

IOTBOT iotbot;

// 🔑 Firebase Configuration | 🔑 Firebase Yapılandırması
#define FIREBASE_PROJECT_URL "https://deneme-961c3-default-rtdb.firebaseio.com/" // Enter the Firebase server URL here. | Buraya Firebase sunucusunun linkini yazınız.
#define FIREBASE_API_KEY "AIzaSyA5wTuxzls6IxQct-G2_hf0i5LWF3ulsec"               // Enter the Firebase server API key here. | Buraya Firebase sunucusunun API anahtarını yazınız.

// 📧 Firebase User Authentication | 📧 Firebase Kullanıcı Kimlik Doğrulama
#define USER_EMAIL "deneme123@gmail.com" // Enter the email address used to create the Firebase user here. | Buraya Firebase sunucusunda oluşturduğunuz kullanıcı e-posta adresini yazınız.
#define USER_PASSWORD "dene123"          // Enter the password for the Firebase user here. | Buraya Firebase sunucusunda oluşturduğunuz kullanıcı şifresini yazınız.

// 📡 WiFi Settings | 📡 WiFi Ayarları
#define WIFI_SSID "WIFI_SSID" // Enter the name of the Wi-Fi network you want to connect to here. | Buraya bağlanmak istediğiniz Wi-Fi ağının adını yazınız.
#define WIFI_PASS "WiFi_PASS" // Enter the password of the Wi-Fi network you want to connect to here. | Buraya bağlanmak istediğiniz Wi-Fi ağının şifresini yazınız.

void setup()
{
    iotbot.begin(); // Initialize IoTBot
    iotbot.serialStart(115200); // Start Serial Communication
    Serial.println("🚀 IoTBot Firebase Example Starting...");

    // LCD Bilgilendirme / LCD Info
    iotbot.lcdShowLoading("Connecting WiFi");
    iotbot.buzzerPlayTone(1000, 200);

    // 🔗 Step 1: Connect to Wi-Fi
    iotbot.wifiStartAndConnect(WIFI_SSID, WIFI_PASS);

    if (!iotbot.wifiConnectionControl())
    {
        iotbot.serialWrite("Device Stopped!");
        iotbot.lcdShowStatus("WiFi Failed", "Retrying...", false);
        while (true) // Endless loop | Sonsuz döngü
        {
            if (iotbot.wifiConnectionControl())
            {
                iotbot.serialWrite("Connection Success! Reworking..");
                break; // Exit the loop, continue.| Döngüden çık, devam et.
            }
        }
    }

    // WiFi Connected
    iotbot.lcdShowStatus("WiFi Connected", "Init Firebase", true);
    iotbot.buzzerPlayTone(1500, 200);
    delay(1000);

    // 🔥 Step 2: Initialize Firebase
    iotbot.lcdShowLoading("Init Firebase");
    iotbot.fbServerSetandStartWithUser(FIREBASE_PROJECT_URL, FIREBASE_API_KEY, USER_EMAIL, USER_PASSWORD);

    // Firebase Ready
    iotbot.lcdShowStatus("Firebase Ready", "Sending Data", true);
    iotbot.buzzerPlayTone(2000, 500);
    delay(1000);

    // ✍️ Step 3: Send Data to Firebase
    iotbot.fbServerSetInt("/device/temperature", 25);
    iotbot.fbServerSetString("/device/status", "Online");
    iotbot.fbServerSetBool("/device/active", true);

    Serial.println("📤 Data sent to Firebase.");
}

void loop()
{
    // 🔄 Step 5: Read Data from Firebase
    int temp = iotbot.fbServerGetInt("/device/temperature");
    String status = iotbot.fbServerGetString("/device/status");
    bool active = iotbot.fbServerGetBool("/device/active");

    // 🖨️ Display Data on Serial Monitor
    Serial.print("🌡️ Temperature: ");
    Serial.println(temp);

    Serial.print("💡 Status: ");
    Serial.println(status);

    Serial.print("🔋 Active: ");
    Serial.println(active ? "Yes" : "No");

    // LCD Geri Bildirim / LCD Feedback
    iotbot.lcdClear();
    iotbot.lcdWriteCR(0, 0, "Temp: " + String(temp));
    iotbot.lcdWriteCR(0, 1, "Status: " + status);
    iotbot.lcdWriteCR(0, 2, "Active: " + String(active ? "Yes" : "No"));
    iotbot.lcdWriteCR(0, 3, "Reading Firebase...");

    delay(60000); // Refresh every 5 seconds
}