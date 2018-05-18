/*
     avrboy  by kimio kosaka (kimio.kosaka@gmail.com)

2018.05.02 ver. 1.0.2
2018.05.02 ver. 1.0.1
2018.05.02 ver. 1.0.0
*/

package main

import (
	"bufio"
	"errors"
	"flag"
	"fmt"
	"log"
	"os"
	"strings"
	"strconv"
	"time"

	"github.com/goburrow/serial"
)

func getArgv() (port string, bps int, device string, file string,  err error) {
	flag.StringVar(&port, "P", "noPort", "string flag")
	flag.IntVar(&bps, "b", 19200, "int flag")
	flag.StringVar(&device, "p", "noDevice", "string flag")
	flag.StringVar(&file, "U", "noFile", "string flag")
	flag.Parse()

	device = strings.ToLower(device)

	switch device {
	case "attiny4":
		device = "ATtiny4"
	case "attiny5":
		device = "ATtiny5"
	case "attiny9":
		device = "ATtiny9"
	case "attiny10":
		device = "ATtiny10"
	case "attiny20":
		device = "ATtiny20"
	case "attiny40":
		device = "ATtiny40"
	default:
		fmt.Println("not support " + device)
		device = "unKnown"
	}
	if port == "noPort" || file == "noFile" || device == "unKnown" {
		err = errors.New("Error! -P "+port+" -b "+strconv.Itoa(bps)+" -U " + file + " -p " + device )
	} else {
		err = nil
	}
	return
}

var (
	address  string
	baudrate int
	databits int    = 8
	stopbits int    = 1
	parity   string = "N"
	message  string
)

func main() {
	// get commandline argument
	address, baudrate, deviceName, hexFileName, errArgv := getArgv()
	// error argument
	if errArgv != nil {
		log.Fatal(errArgv)
	}

	// Open hex file
	fd, ferr := os.Open(hexFileName)
	if ferr != nil {
		log.Fatal(ferr)
	}
	// Close hex file
	defer fd.Close()

  // serial port config
	config := serial.Config{
		Address:  address,
		BaudRate: baudrate,
		DataBits: databits,
		StopBits: stopbits,
		Parity:   parity,
		Timeout:  10 * time.Second,
	}

	// open sereialPort
	port, err := serial.Open(&config)
	if err != nil {
		log.Printf("open %s %v", address, err)
		log.Fatal()
	}

	// close serialPortdefer
	defer func() {
		err := port.Close()
		if err != nil {
			log.Fatal(err)
		}
	}()

  // send deviceID request to programmer
	time.Sleep(800 * time.Millisecond)
	msg := "I"
	if _, err = port.Write([]byte(msg)); err != nil {
		log.Fatal(err)
	}

	if baudrate > 9600 {
  	time.Sleep(200 * time.Millisecond)
		if _, err = port.Write([]byte(msg)); err != nil {
			log.Fatal(err)
		}
	}

	// read programmer respons message
	oneByteBuffer := make([]byte, 1)
	reciveCharBuffer := make([]byte, 255)
	charCounter := 0
	for oneByteBuffer[0] != '>' {
		if _, err = port.Read(oneByteBuffer); err != nil {
			log.Fatal(err)
		}
		reciveCharBuffer[charCounter] = oneByteBuffer[0]
		charCounter++
	}

  // programmers' message (byte slice) to string arry
	var reciveStringBuffer [255]string
	var lineCounter int = 0
	for i := 0; i < charCounter; i++ {
		switch {
		case reciveCharBuffer[i] >= ' ' && reciveCharBuffer[i] <= '~':
			switch {
			case i == 0:
				reciveStringBuffer[lineCounter] = string(reciveCharBuffer[i])
			default:
				reciveStringBuffer[lineCounter] = reciveStringBuffer[lineCounter] + string(reciveCharBuffer[i])
			}
		case reciveCharBuffer[i] == '\n':
			lineCounter++
		default:
		}
	}

	// read programmer's starting message
	attinyConnectedPosition := 0
	deviceConnectedFlag := false
	for i := 0; i <= lineCounter; i++ {
		if strings.Contains(reciveStringBuffer[i], "ATtiny") {
			attinyConnectedPosition = i
		}
		if !deviceConnectedFlag {
			deviceConnectedFlag = reciveStringBuffer[i] == (deviceName + " connected")
		}
	}

	// check connected Attiny device
	if !deviceConnectedFlag {
		log.Fatal("Error! Your request is " + deviceName + ", but " + reciveStringBuffer[attinyConnectedPosition])
	}
	fmt.Printf(reciveStringBuffer[attinyConnectedPosition])

	/*****  programming ATtiny  *****/
	fmt.Println(" / Write " + hexFileName)
	// set scanner for hex file
	scanner := bufio.NewScanner(fd)
	// send programming command to ATtiny programmer
	cmd := "P "
	if _, err = port.Write([]byte(cmd)); err != nil {
		log.Fatal(err)
	}
	// send hex data to ATtiny programmer
	var lineBuffer string
	for scanner.Scan() {
		lineBuffer = scanner.Text()
		if _, err = port.Write([]byte(lineBuffer)); err != nil {
			log.Fatal(err)
		}
		time.Sleep(50 * time.Millisecond)
	}
	if ferr = scanner.Err(); ferr != nil {
		log.Fatal(ferr)
	}

	// read ATtiny programmer's respons message
	oneByteBuffer[0] = ' '
	for oneByteBuffer[0] != '>' {
		if _, err = port.Read(oneByteBuffer); err != nil {
			log.Fatal(err)
		}
		fmt.Printf("%c", oneByteBuffer[0])
	}
	fmt.Println()
	return
}
