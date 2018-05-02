package main

import (
	"bufio"
	"errors"
	"flag"
	"fmt"
	"log"
	"os"
	"strings"
	"time"

	"github.com/goburrow/serial"
)

func getArgv() (port string, bps int, device string, file string, err error) {
	flag.StringVar(&port, "P", "noPort", "string flag")
	flag.IntVar(&bps, "b", 9600, "int flag")
	flag.StringVar(&device, "p", "noDevice", "string flag")
	flag.StringVar(&file, "U", "noFile", "string flag")
	flag.Parse()
	switch device {
	case "t4":
		device = "ATtiny4"
	case "t5":
		device = "ATtiny5"
	case "t9":
		device = "ATtiny9"
	case "t10":
		device = "ATtiny10"
	case "t20":
		device = "ATtiny20"
	case "t40":
		device = "ATtiny40"
	default:
		device = "unKnown"
	}
	if port == "noPort" || file == "noFile" || device == "unKnown" {
		err = errors.New("Error! -P or -U or -p")
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

	config := serial.Config{
		Address:  address,
		BaudRate: baudrate,
		DataBits: databits,
		StopBits: stopbits,
		Parity:   parity,
		Timeout:  30 * time.Second,
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
		log.Fatal("Error! Your setting " + deviceName + ", but " + reciveStringBuffer[attinyConnectedPosition])
	}
	fmt.Println(reciveStringBuffer[attinyConnectedPosition])

	/*****  programming ATtiny  *****/
	fmt.Println("Write " + hexFileName)
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
