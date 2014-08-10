package main

import (
	//	"encoding/json"
	"bufio"
	"flag"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"path"
	"strconv"
	"strings"
	"sync"
	//	"code.google.com/p/go.net/websocket"
	"github.com/tarm/goserial"
	//"github.com/LHSRobotics/gdmux/pkg/staubli"
	"github.com/LHSRobotics/gdmux/pkg/vplus"
)

type point struct {
	x, y, z, a, b, c float64
}

var origin point

var (
	// armPort is the serial file connected to the arm controller's data line. For the Staubli
	// its baudrate 19200, we assume that's already set for the device file. (I.e. with stty.)
	ttyData     = flag.String("datatty", "/dev/ttyS0", "serial tty to the Staubli data line")
	baudData    = flag.Int("datarate", 19200, "baud rate for the staubli's data line")
	ttyConsole  = flag.String("consoletty", "/dev/ttyUSB0", "serial tty to the Staubli console prompt")
	baudConsole = flag.Int("consolerate", 38400, "baud rate for the staubli's console")

	originx = flag.Float64("x", 500, "x coordinates for the origin")
	originy = flag.Float64("y", 0, "y coordinates for the origin")
	originz = flag.Float64("z", -100, "z coordinates for the origin")

	use_dummy = flag.Bool("dummy", false, "don't actually send commands to the arm")
	httpAddr  = flag.String("http", "", "tcp address on which to listen")
	sendvplus = flag.Bool("sendv", false, "send over the V+ code on startup")
	verbose   = flag.Bool("verbose", false, "print lots output")

	arm     Arm
	running = false
)

type Arm interface {
	Move6DOF(x, y, z, yaw, pitch, roll float64) error
}

type Staubli struct {
	rw io.ReadWriter
	sync.Mutex
	cur    point
	reader *bufio.Reader
}

type dummy struct{}

var Dummy = &dummy{}

func (s *dummy) Move6DOF(x, y, z, yaw, pitch, roll float64) error {
	log.Printf("Move6DOF %.3f %.3f %.3f %.3f %.3f %.3f\r\n", x, y, z, yaw, pitch, roll)
	return nil
}

// Move the arm to the point (x,y,z), without guaranteeing a staight line.
func (s *Staubli) Move6DOF(x, y, z, yaw, pitch, roll float64) error {
	// we probably need a lock here...
	_, err := fmt.Fprintf(s.rw, "9 %.3f %.3f %.3f %.3f %.3f %.3f\r\n", x, y, z, yaw, pitch, roll)
	if err != nil {
		return fmt.Errorf("error sending coordinates to arm: %s", err)
	}

	if r := s.readReply(); !strings.HasPrefix(r, "OK") {
		return fmt.Errorf("error from arm: %s", r)
	}
	return nil
}

func (s *Staubli) readReply() string {
	line, err := s.reader.ReadString('\n')
	if err != nil {
		log.Println("error reading ack from arm: ", err)
	}

	line = strings.TrimSpace(line)
	if line == "" {
		return s.readReply()
	}
	return line
}

func NewStaubli(rw io.ReadWriter) *Staubli {
	a := &Staubli{
		rw:     rw,
		reader: bufio.NewReader(rw),
	}

	return a
}

func handleRun(w http.ResponseWriter, r *http.Request) {
	// TODO: communicate the running state to js, so the right buttons get enabled/disabled.
	// if running {
	// 	log.Printf("Got run request from %s, but the arm is already running.\n", r.RemoteAddr)
	// 	return
	// }
	log.Printf("Got a request from %s\n", r.RemoteAddr)
	//sessionLock.Lock()
	running = true
	//fmt.Println("comingata")

	//log.Println("request Method: ", r.Method)
	//log.Println("request Body: ", r.Body)
	//log.Println("request Header: ", r.Header)
	//log.Println("request URL: ", r.URL)
	//log.Println("request query: ", r.URL.Query())

	v := r.URL.Query()
	log.Println(
		"x", v.Get("x"),
		"y", v.Get("y"),
		"z", v.Get("z"),
		"yaw", v.Get("yaw"),
		"pitch", v.Get("pitch"),
		"roll", v.Get("roll"))

	x, _ := strconv.ParseFloat(v.Get("x"), 32)
	y, _ := strconv.ParseFloat(v.Get("y"), 32)
	z, _ := strconv.ParseFloat(v.Get("z"), 32)
	yaw, _ := strconv.ParseFloat(v.Get("yaw"), 32)
	pitch, _ := strconv.ParseFloat(v.Get("pitch"), 32)
	roll, _ := strconv.ParseFloat(v.Get("roll"), 32)

	//log.Printf("RUNNING GCODE!\n")
	//dmux(r.Body)
	err := arm.Move6DOF(x+origin.x, y+origin.y, z+origin.z,
		yaw, pitch, roll)

	if err != nil {
		log.Printf(" → %s\n", err)
	}

	running = false
	//sessionLock.Unlock()

	log.Printf("Done.\n")
}

func sendPg() {
	log.Println("Sending over V+ code")
	s, err := serial.OpenPort(&serial.Config{Name: *ttyConsole, Baud: *baudConsole})
	if err != nil {
		log.Fatal(err)
	}
	defer s.Close()
	console := vplus.NewConsole(s)

	f := strings.Split(os.Getenv("GOPATH"), ":")[0] + "/src/github.com/coolvision/arm6dof/gcode.pg"
	log.Println("Send file: ", f)

	err = console.Cmd("abort")
	if err != nil {
		log.Fatal("error sending file: ", f)
	}
	console.Expect()
	err = console.Cmd("kill")
	if err != nil {
		log.Fatal("error sending file: ", f)
	}
	console.Expect()
	err = console.UpdateFile(f)
	if err != nil {
		log.Fatal("error sending file: ", f)
	}
	err = console.Cmd(fmt.Sprintf("ex %s", path.Base(f)))
	if err != nil {
		log.Fatal("error sending file: ", f)
	}
}

func initArm() {
	origin.x, origin.y, origin.z = *originx, *originy, *originz

	if *use_dummy {
		arm = Dummy
	} else {
		log.Println("Opening ", *ttyData)
		s, err := serial.OpenPort(&serial.Config{Name: *ttyData, Baud: *baudData})
		if err != nil {
			log.Fatal(err)
		}
		arm = NewStaubli(s)
	}

	if *sendvplus {
		sendPg()
	}
}

func main() {
	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "usage: %s -http :5000\n", os.Args[0])
		flag.PrintDefaults()
	}
	flag.Parse()

	log.Println("sendvplus", *sendvplus)

	if *httpAddr != "" {
		//clients.m = make(map[chan string]bool)
		initArm()
		log.Println("Listening on ", *httpAddr)
		http.HandleFunc("/run", handleRun)
		log.Fatal(http.ListenAndServe(*httpAddr, nil))
	}
}
