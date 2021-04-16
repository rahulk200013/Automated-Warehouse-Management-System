function doGet(e){
  
  var ss = SpreadsheetApp.getActive();

  var sheet = ss.getSheetByName(e.parameter["id"]);

  var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];

  var lastRow = sheet.getLastRow();

  var cell = sheet.getRange('a1');
  var col = 0;
  var d = new Date();
  dateString = Utilities.formatDate(d, "GMT+5:30", 'EEE MMM dd, yyyy HH:mm:ss')

  for (i in headers){

    // loop through the headers and if a parameter name matches the header name insert the value

    if (headers[i] == "Timestamp")
    {
      val = dateString;
    }
    else
    {
      val = e.parameter[headers[i]]; 
    }

    // append data to the last row
    cell.offset(lastRow, col).setValue(val);
    col++;
  }

  my_email_id = "eyrc.vb.0574@gmail.com";
  eyantra_email_id = "eyrc.vb.0000@gmail.com";

  days_to_deliver = "1";

  if (e.parameter["Item"] == "Medicine"){
    days_to_deliver = "1";
  } else if (e.parameter["Item"] == "Food"){
    days_to_deliver = "3";
  } else {
    days_to_deliver = "5";
  }

  if (e.parameter["id"] == "OrdersDispatched")
  {
    var message = "Dear Customer,\n" +
                  "Your order has been dispatched from our warehouse.\n\n" +
                  "Here is the summary of your order: \n\n" + 
                  "Order ID: " + e.parameter["Order ID"] +"\n" +
                  "Item: " + e.parameter["Item"] +"\n" + 
                  "Quantity: " + e.parameter["Dispatch Quantity"] +"\n" +
                  "Cost: " + e.parameter["Cost"] +"\n" + 
                  "City: " + e.parameter["City"] +"\n" + 
                  "Dispatch Date and Time: " + e.parameter["Dispatch Date and Time"] + " GMT + 5:30" + "\n\n" +
                  "We will inform you once the order is shipped\n\n" +
                  "Feel free to reach out to us in case you have any enquiry"

    MailApp.sendEmail(my_email_id, " Your Order has been Dispatched! ", message);
    MailApp.sendEmail(eyantra_email_id, " Your Order has been Dispatched! ", message);
  } else if (e.parameter["id"] == "OrdersShipped")
  {
    var message = "Dear Customer,\n" +
                  "Your order has been shipped from our warehouse. It will be drone delivered to you in " + days_to_deliver + " day(s). \n\n" +
                  "Here is the summary of your order: \n\n" +
                  "Order ID: " + e.parameter["Order ID"] +"\n" +
                  "Item: " + e.parameter["Item"] +"\n" +
                  "Quantity: " + e.parameter["Shipped Quantity"] +"\n" +
                  "Cost: " + e.parameter["Cost"] +"\n" +
                  "City: " + e.parameter["City"] +"\n" +
                  "Shipped Date and Time: " + e.parameter["Shipped Date and Time"] + " GMT + 5:30" + "\n" +
                  "Estimated Time of Delivery: " + e.parameter["Estimated Time of Delivery"] + "\n\n" +
                  "Feel free to reach out to us in case you have any enquiry"

    MailApp.sendEmail(my_email_id, " Your Order has been Shipped! ", message);
    MailApp.sendEmail(eyantra_email_id, " Your Order has been Shipped! ", message);
  }

  return ContentService.createTextOutput('success');
}

function emailQuota(){
    var emailQuotaRemaining = MailApp.getRemainingDailyQuota();
    Logger.log("Remaining email quota: " + emailQuotaRemaining);
}