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

network DADCA_ACK_n20_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=66.45,32.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=160.40,65.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=140.55,3.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=36.46,136.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=90.65,247.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=11.91,223.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=178.26,140.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=162.69,244.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=222.97,314.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=158.25,384.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=218.19,405.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=285.18,339.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=331.68,396.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=337.93,303.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=383.01,251.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=439.60,208.58");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=488.84,150.06");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=503.21,72.87");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=533.96,1.81");
    }

}

