//-----------------------------------------------------------------------------
// Header
// F() macro saves strings to flash memory instead of SRAM
//-----------------------------------------------------------------------------
void append_page_header() {
    // Header
    webpage  = F("<!DOCTYPE html><html>");
    webpage += F("<head>");
    webpage += F("<title>VAMoS Data Download Page</title>"); // NOTE: 1em = 16px
    webpage += F("<meta name='viewport' content='user-scalable=yes,initial-scale=1.0,width=device-width'>");

    // Styling
    webpage += F("<style>");

    webpage += F("html {");
        webpage += F("font-family:Helvetica;");
        webpage += F("display: inline-block;");
        webpage += F("margin: 0px auto;");
        webpage += F("text-align: center; }");

    webpage += F(".button {");
        webpage += F("background-color: #4CAF50;");
        webpage += F("border: none;");
        webpage += F("color: white;");
        webpage += F("padding: 16px 40px;");
        webpage += F("text-decoration: none;");
        webpage += F("font-size: 30px;");
        webpage += F("margin: 2px;");
        webpage += F("cursor: pointer;}");
    webpage += F("</style></head>");

    // Body
    webpage += F("<body>");
        webpage += F("<h1>VAMoS Data Download Page</h1>");
        webpage += F("<p>");
        webpage += F("<a href=\"/download\">");
        webpage += F("<button class=\"button\">Download</button>");
        webpage += F("</a>");
        webpage += F("</p>");
}

//-----------------------------------------------------------------------------
// Footer
// Probably won't be needed
//-----------------------------------------------------------------------------
void append_page_footer(){ // Saves repeating many lines of code for HTML page footers
    webpage += F("</body></html>");
}
