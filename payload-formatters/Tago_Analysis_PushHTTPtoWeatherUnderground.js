/*
 ** Analysis 
 ** Submit observation to WeatherUnderground using Get to HTTP Route
 ** Derived from similar script for payload-formatters\Tago_Analysis_PushToWOW.js
 */

 const { Analysis, Device, Utils } = require("@tago-io/sdk");
 const axios = require("axios");
 
 async function getToHTTP(context) {
 
     // reads the values from the environment and saves it in the variable env_vars
   const env_vars = Utils.envToJson(context.environment);
   if (!env_vars.device_token) {
     return context.log("Device token not found on environment parameters");
   }
 
   const device = new Device({ token: env_vars.device_token });
  var pv_production = 0;
  var radiance = 0;
  var total_2_days = 0;

    //  Fields of the TTN payload as captured in Device
  const dataFields = ["field1","field2","humidity2","field4","field5","field6","field7","field8","field9","rain_yesterday"];

    // create the filter options to get the last-entered set of obs data from TagoIO
   const filter = {
     variables: dataFields,
     query: "last_item",
   };
 
     const resultArray = await device.getData(filter).catch(() => null);
 
   //  for (const index in dataFields) {
   //   context.log(`${dataFields[index]} :: ${resultArray[index].value} \t ${resultArray[index].serie}`);
   //  };
 
   // Check if the array is not empty
   if (!resultArray || !resultArray[0]) {
     return context.log("Empty Array for last observations")
   };
   
     // Update 2-day rain total with latest accumulation
     const yesterday_total = resultArray[9].value;
     total_2_days = yesterday_total + resultArray[7].value;
  
   //  Prepare formula for Dew Point calculation
   const tempMeasured = resultArray[1].value;
   const humidityMeasured = resultArray[2].value;
   const alpha = Math.log(humidityMeasured/100) + 17.62 * tempMeasured / (243.12 + tempMeasured);

 
   //  Reset wind direction to report in range {0,360} degrees from Nth
     var stdWindDirn = resultArray[6].value;

     if (stdWindDirn < 0) {
         stdWindDirn = 360 + stdWindDirn;
        } else if (stdWindDirn > 360) {
         stdWindDirn = stdWindDirn - 360;
   }

   context.log(resultArray[0].time);
  
 //  Query solarEdge web system for current panel production power.
 const solar_options = {
  url: "https://monitoringapi.solaredge.com/site/2037326/currentPowerFlow.json",
  method: "GET",
//   headers: {
//     Authorization: "Your-Account-Token",
//   },
  // How to use HTTP QueryString
  params: {
    api_key: env_vars.solar_key
  //  serie: 123,
 },
  //
  // How to send a HTTP Body:
  // body: 'My text body',
};

try {
  const solar_result = await axios(solar_options);
  context.log(solar_result.data);

  // Assemble Solar Radiation data for insert to Bucket  
  // Use the reommissioned Device whose Payload Parser has been cleared.

  pv_production = solar_result.data.siteCurrentPowerFlow.PV.currentPower;
  radiance = pv_production * env_vars.normalisation_factor;
  //  context.log("Current Power Production (",solar_result.data.siteCurrentPowerFlow.unit,")", solar_result.data.siteCurrentPowerFlow.PV.currentPower);
  
  if (solar_result.data.siteCurrentPowerFlow.unit != "kW") {
      context.log("Units not kW");
      context.log(radiance);
  } 

  // update Bucket with calculated radiance & 2 day rain values
  const resultSend = await device.sendData([
     {
     variable: "total_rain_2_days",
     value: total_2_days,
     },
     {
     variable: "solarad",
     value: radiance,
     },
   ]);

  context.log(resultSend); context.log(radiance); context.log("\n");
  

} catch (error) {
  context.log(`${error}\n${error}`);
}

 
 
  //   construct the POST with query parameters as per WU requirements (imperial units)
   const options = {
     url: "https://rtupdate.wunderground.com/weatherstation/updateweatherstation.php",
     method: "GET",
   //  headers: {
    //   Authorization: "Your-Account-Token",
   //  },
     // HTTP QueryString
      params: {
       ID: env_vars.siteid,
       PASSWORD: env_vars.siteAuthenticationKey,
       action: "updateraw",
       dateutc: "now",
      // dateutc: resultArray[0].time,
       tempf: resultArray[1].value * 9/5 +32,
       humidity: resultArray[2].value,
       baromin: (resultArray[3].value*1.01254 - 1.7)/33.864,
       rainin: resultArray[4].value/25.4,
       windspeedmph: resultArray[5].value/1.609,
       winddir: stdWindDirn,
       dailyrainin: resultArray[7].value/25.4,
       dewptf: (alpha * 243.12/(17.62 - alpha)) * 9/5 +32,
       windgustmph: resultArray[0].value/1.609,
       windgustdir: resultArray[8].value
    //   softwaretype: env_vars.softwaretype
       },
     //
     // How to send a HTTP Body:
     // body: 'My text body',
   };
 
   try {
     const result = await axios(options);    context.log(result.data);  } catch (error) {    context.log(`${error}`);
   }
 }
 
 module.exports = new Analysis(getToHTTP);
 
 // To run analysis on your machine (external)
 // module.exports = new Analysis(getToHTTP, { token: "YOUR-TOKEN" });