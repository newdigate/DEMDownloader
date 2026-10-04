#include <iostream>
#include <semaphore>
#include <thread>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#else
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#endif
#include "include/shader.h"

#include "tiffio.h"
#include "httplib.h"
#include "ctpl_stl.h"  // or <ctpl_stl.h> if ou do not have Boost library

static const char* vertexShaderCode = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in vec2 aTexCoord;

out vec3 ourColor;
out vec2 TexCoord;

void main()
{
	gl_Position = vec4(aPos, 1.0);
	ourColor = aColor;
	TexCoord = vec2(aTexCoord.x, aTexCoord.y);
}
)glsl";

static const char* fragmentShaderCode = R"glsl(
#version 330 core
out vec4 FragColor;

in vec3 ourColor;
in vec2 TexCoord;

// texture sampler
uniform sampler2D texture1;
uniform int maxValue;

void main()
{
	FragColor = texture(texture1, TexCoord);
    if (FragColor.r == 0 && FragColor.g == 0 && FragColor.b == 0xFF)
    {
    } else
    {
        FragColor.r = 255 * FragColor.r / maxValue;
        FragColor.g = 255 * FragColor.g / maxValue;
        FragColor.b = 255 * FragColor.b / maxValue;
    }
}
)glsl";

using namespace std::chrono_literals;

#define CA_CERT_FILE "../ca-bundle.crt"

GLFWwindow *window;
GLuint texture;
unsigned int VBO, VAO, EBO;
Shader *shader;
uint16_t* raster;
unsigned char maxDisplayValue = 1;
unsigned char* displayTexture;

int sampleperpixel = 3, bitspersample = 16;
int gridSizeWidth =	2048, gridSizeHeight = 1792;

bool load_tiff(const char* filename, int &spp, int &width, int &height, int &bps, uint16_t* &r) {
    TIFF* tif = TIFFOpen(filename, "r") ;
    if (tif) {
        TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
        TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
        TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &spp);
        TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bps);
        int64 npixels = gridSizeWidth * gridSizeHeight;
        int bytesPerSample = bitspersample / 8;
        r = (uint16_t*) _TIFFmalloc(npixels * (bitspersample/8) * sampleperpixel);
        for (auto row = 0; row < gridSizeHeight; row++)
            TIFFReadScanline(tif, &(r[row * gridSizeWidth * bytesPerSample/2 * sampleperpixel]), row);

        TIFFClose(tif);
        return true;
    }
    return false;
}

void save_tiff(const char* filename, uint16_t* rasterTarget) {
    TIFF *outnew = TIFFOpen(filename, "w");

    TIFFSetField (outnew, TIFFTAG_IMAGEWIDTH, gridSizeWidth);  // set the width of the image

    TIFFSetField(outnew, TIFFTAG_IMAGELENGTH, gridSizeHeight);    // set the height of the image
    TIFFSetField(outnew, TIFFTAG_SAMPLESPERPIXEL, sampleperpixel);   // set number of channels per pixel
    TIFFSetField(outnew, TIFFTAG_BITSPERSAMPLE, bitspersample);    // set the size of the channels
    TIFFSetField(outnew, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);    // set the origin of the image.
    tsize_t linebytes = sampleperpixel * gridSizeWidth * 2;     // length in memory of one row of pixel in the image.

    //   Some other essential fields to set that you do not have to understand for now.
    TIFFSetField(outnew, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(outnew, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);


    unsigned char *buf = NULL;        // buffer used to store the row of pixel information for writing to file
    //    Allocating memory to store the pixels of current row
    if (TIFFScanlineSize(outnew)!=linebytes)
        buf =(unsigned char *)_TIFFmalloc(linebytes);
    else
        buf = (unsigned char *)_TIFFmalloc(TIFFScanlineSize(outnew));

    // We set the strip size of the file to be size of one row of pixels
    TIFFSetField(outnew, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(outnew, linebytes));
    //Now writing image to the file one strip at a time
    for (uint32 row = 0; row <  gridSizeHeight; row++)
    {
        memcpy(buf, &rasterTarget[row*linebytes/2], linebytes);    // check the index here, and figure out why not using h*linebytes
        if (TIFFWriteScanline(outnew, buf, row, 0) < 0)
            break;
    }
    TIFFClose(outnew);
}

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
}

void refresh() {
    glfwPollEvents();
    glfwMakeContextCurrent(window);

    glClearColor(0.2f, 0.3f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, gridSizeWidth, gridSizeHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, displayTexture);

    shader->use();
    shader->setInt("maxValue", maxDisplayValue);

    // render container
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
    // -------------------------------------------------------------------------------
    glfwSwapBuffers(window);
    glfwPollEvents();
    //int microsStop = micros();
    //Serial.printf("OpenGL update: %u (%i micros)\n", _updateCount, microsStop - microsStart);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

bool initOpenGl() {
    /* Initialize the library */
    if (!glfwInit()) {
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWmonitor *monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode *mode = glfwGetVideoMode(monitor);
    window = glfwCreateWindow(640, 480, "Elevation Download", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    std::string vertexShaderCodeStr = std::string(vertexShaderCode);
    std::string fragmentShaderCodeStr = std::string(fragmentShaderCode);

    shader = new Shader(vertexShaderCodeStr,  fragmentShaderCodeStr );

    float vertices[] = {
            // positions          // colors           // texture coords
            1.0f,  1.0f, 0.0f,   1.0f, 0.0f, 0.0f,   1.0f, 0.0f, // top right
            1.0f, -1.0f, 0.0f,   0.0f, 1.0f, 0.0f,   1.0f, 1.0f, // bottom right
            -1.0f, -1.0f, 0.0f,   0.0f, 0.0f, 1.0f,   0.0f, 1.0f, // bottom left
            -1.0f,  1.0f, 0.0f,   1.0f, 1.0f, 0.0f,   0.0f, 0.0f  // top left
    };
    unsigned int indices[] = {
            0, 1, 3, // first triangle
            1, 2, 3  // second triangle
    };

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // texture coord attribute
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);


    // load and create a texture
    // -------------------------

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture); // all upcoming GL_TEXTURE_2D operations now have effect on this texture object
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);	// set texture wrapping to GL_REPEAT (default wrapping method)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    return true;
}

void rainbow(double value, unsigned char &r, unsigned char &g, unsigned char &b) {
    /*convert to long rainbow RGB*/
    double a= (1.0f-value)/0.2;
    unsigned char X= floor(a);
    unsigned char Y= floor(255.0*(a-X));
    switch(X)
    {
        case 0: r=255;g=Y;b=0;break;
        case 1: r=255-Y;g=255;b=0;break;
        case 2: r=0;g=255;b=Y;break;
        case 3: r=0;g=255-Y;b=255;break;
        case 4: r=Y;g=0;b=255;break;
        case 5: r=255;g=0;b=255;break;
    }
}

int main(int argc, char** argv)
{
    signal(SIGPIPE, signalHandler);
    //double north = -34.070972, east=18.413194, south=-33.979583, west=18.303194;
    // table mountain: double north = -34.0162500000124624 , east=18.5231944444707395, south=-33.8701388889013302, west=18.3251388889151592;
    double north =-34.2163446879999995, east=18.4625244140000007, south=-34.1845418309999971, west=18.4185791019999989;

    volatile bool finished = false;
    unsigned long startY = 0;
    if (!load_tiff("../simonstown.tif", sampleperpixel, gridSizeWidth, gridSizeHeight, bitspersample, raster )) {
        tmsize_t npixels = gridSizeWidth * gridSizeHeight;
        raster = (uint16_t*) _TIFFmalloc(npixels * sampleperpixel * bitspersample/8);
    } else
    {   // figure out where we left off, where we need to start

        unsigned long index = 0;
        for (auto y=0; y<gridSizeHeight; y++) {
            for (auto x=0; x<gridSizeWidth; x++) {
                uint16_t rasterValue = raster[index*3];
                if (rasterValue > 0) {
                    startY = y+1;
                    unsigned char rasterDisplayValue = (rasterValue >> 8);
                    if (rasterDisplayValue > maxDisplayValue ) {
                        maxDisplayValue = rasterDisplayValue;
                    }
                }
                index++;
            }
        }
    }

    std::cout << "starting at y: " << std::to_string(startY) << std::endl;
    std::cout << "maxDisplayValue so far: " << std::to_string(maxDisplayValue) << std::endl;

    displayTexture = new unsigned char [gridSizeWidth * gridSizeHeight * 3];
    unsigned long index = 0;
    for (auto y=0; y<gridSizeHeight; y++) {
        for (auto x=0; x<gridSizeWidth; x++) {
            if (y < startY && raster[index] == 0) {
                displayTexture[index++] = 0;
                displayTexture[index++] = 0;
                displayTexture[index++] = 0xFF;
            } else {
                unsigned char r, g, b;
                rainbow(raster[index] / 65535.0, r, g, b);
                displayTexture[index++] = r;
                displayTexture[index++] = g;
                displayTexture[index++] = b;
            }
        }
    }

    if (!initOpenGl())
        return -1;

    std::mutex g_i_mutex;  // protects g_i
    ctpl::thread_pool pool(50);

    std::thread downloadThread([&finished, north, east, south, west, &pool, &g_i_mutex, startY] () {

        for ( auto y=startY; y < gridSizeHeight; y++) {

            std::vector<std::future<void>> results;
            std::vector<std::string*> urls;

            for ( auto x = 0; x < gridSizeWidth; x++) {
                auto *url = new std::string("/erdas-iws/ogc/wms/GeoSpatial%20Datasets?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetFeatureInfo&BBOX=");
                url->append(std::to_string(north));
                url->append("%2C");
                url->append(std::to_string(west));
                url->append("%2C");
                url->append(std::to_string(south));
                url->append("%2C");
                url->append(std::to_string(east));
                url->append("&CRS=EPSG%3A4326&WIDTH=");
                url->append(std::to_string(gridSizeWidth));
                url->append("&HEIGHT=");
                url->append(std::to_string(gridSizeHeight));
                url->append("&LAYERS=Elevation_CCT_GLM2019_5m_DEM&STYLES=&FORMAT=image%2Fpng&QUERY_LAYERS=Elevation_CCT_GLM2019_5m_DEM&INFO_FORMAT=text%2Fplain&I=");
                url->append( std::to_string(x) );
                url->append( "&J=" );
                url->append(std::to_string(y ));
                urls.push_back(url);

                uint64_t index = gridSizeWidth * y + x;
                displayTexture[index * 3] = 0xFF;
                displayTexture[index * 3 + 1] = 0;
                displayTexture[index * 3 + 2] = 0;

                results.push_back( pool.push(
                        [ &g_i_mutex ] (int id, std::string *url, int x, int y) {
                            double currentValue = 0.0;

                            auto *cli = new httplib::SSLClient("cityimg.capetown.gov.za");
                            cli->set_ca_cert_path(CA_CERT_FILE);
                            cli->enable_server_certificate_verification(false);

                            uint64_t index = gridSizeWidth * y + x;
                            displayTexture[index * 3] = 0;
                            displayTexture[index * 3 + 1] = 0xFF;
                            displayTexture[index * 3 + 2] = 0;

                            int attempt = 0; int maxAttempts = 500;
                            while (attempt < maxAttempts) {
                                attempt++;

                                httplib::Result r = cli->Get(*url);
                                if (r && r->status == httplib::StatusCode::OK_200) {
                                    size_t pos = r->body.find("BAND 0:");
                                    if (pos != std::string::npos) {
                                        std::string band0 = r->body.substr(pos + 8, 5);
                                        currentValue = std::stod(band0);
                                    } else {
                                        currentValue = 0.0;
                                        const std::lock_guard<std::mutex> lock(g_i_mutex);
                                        std::cout << "no value: " <<  r->body << std::endl;
                                    }
                                    break;
                                } else {
                                    const std::lock_guard<std::mutex> lock(g_i_mutex);
                                    std::cout << "attempt " << std::to_string(attempt) << " - x: " << std::to_string(x) << " y: " << std::to_string(y) << std::endl;
                                }
                            }

                            {
                                const std::lock_guard<std::mutex> lock(g_i_mutex);
                                std::cout << "x: " << std::to_string(x) << " y: " << std::to_string(y) << " - " << currentValue << std::endl;
                            }

                            if (currentValue < 0.0)
                                currentValue = 0.0;

                            //uint16_t demValue = round(65535.0 * currentValue / 1100.0);
                            uint16_t demValue = round(65535.0 * currentValue / 700.0);

                            raster[index * 3] = demValue;
                            raster[index * 3 + 1] = demValue;
                            raster[index * 3 + 2] = demValue;

                            if (currentValue == 0.0) {
                                displayTexture[index * 3] = 0;
                                displayTexture[index * 3 + 1] = 0;
                                displayTexture[index * 3 + 2] = 0xFF;
                            } else {
                                unsigned char r, g, b;
                                rainbow(currentValue / 700.0, r, g, b);
                                displayTexture[index * 3] = r;
                                displayTexture[index * 3 + 1] = g;
                                displayTexture[index * 3 + 2] = b;

                                unsigned char bit8value = round(255.0 * currentValue / 1100.0);
                                //displayTexture[index * 3] = 0;
                                //displayTexture[index * 3 + 1] = bit8value;
                                //displayTexture[index * 3 + 2] = 0;
                                if (bit8value > maxDisplayValue) {
                                    maxDisplayValue = bit8value;
                                }
                            }

                            delete cli;
                        }, url, x, y));

            }

            for ( auto &result : results)
                result.get();

            save_tiff("../simonstown.tif", raster);

            for ( auto url : urls)
                delete url;
        }

        finished = true;

    });
    std::chrono::milliseconds lastRefresh = 0ms;

    while (!glfwWindowShouldClose(window) && !finished) {
        std::chrono::milliseconds now = duration_cast< std::chrono::milliseconds >( std::chrono::system_clock::now().time_since_epoch() );
        if (now - lastRefresh > 500ms) {
            refresh();
            lastRefresh = now;
        }

        std::this_thread::sleep_for(0.05s);
    }
    glfwDestroyWindow(window);
    delete shader;
    delete displayTexture;
    _TIFFfree(raster);
    glfwTerminate();
}