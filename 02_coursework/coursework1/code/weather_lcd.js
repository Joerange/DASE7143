var Client = require('node-rest-client').Client;
var client = new Client();
var execFile = require('child_process').execFile;

// Use the same city weather URL as weather.js.
var weatherUrl = "http://api.openweathermap.org/data/2.5/weather?q=Hong%20Kong&APPID=344be8de63bac0b47dd9a4d663cf1b34";

client.get(weatherUrl, function (data, response) {
    if (response.statusCode !== 200 || !data.main ||
            typeof data.main.temp !== 'number') {
        console.error('Unable to read the city temperature.');
        return;
    }

    // OpenWeather returns Kelvin by default; also support URLs with units set.
    var temperature = data.main.temp;
    if (/[?&]units=imperial(?:&|$)/i.test(weatherUrl)) {
        temperature = (temperature - 32) * 5 / 9;
    } else if (!/[?&]units=metric(?:&|$)/i.test(weatherUrl)) {
        temperature -= 273.15;
    }

    var displayText = 'Temp: ' + temperature.toFixed(1) + ' C';
    console.log(displayText);
    execFile('/home/debian/lcd_display', [displayText], function (error) {
        if (error) {
            console.error('Unable to update the LCD: ' + error.message);
        }
    });
}).on('error', function (error) {
    console.error('Weather request failed: ' + error.message);
});

