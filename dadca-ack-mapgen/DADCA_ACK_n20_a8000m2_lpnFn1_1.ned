package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n20_a8000m2_lpnFn1_1
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=7.23,105.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=140.13,192.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=189.93,150.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=38.49,34.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=35.92,184.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=209.51,65.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=109.25,32.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=83.52,270.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=130.43,337.01");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=68.68,393.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=262.55,243.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=288.69,3.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=252.27,112.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=312.41,74.26");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=378.89,68.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=472.09,95.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=413.20,120.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=472.34,18.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=543.21,62.51");
    }

}

