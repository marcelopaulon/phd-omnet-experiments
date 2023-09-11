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

network DADCA_ACK_n20_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=124.67,4.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=32.43,10.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=11.08,100.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=101.21,206.16");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=189.11,225.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=121.89,264.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=111.23,334.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=183.34,349.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=255.94,360.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=335.25,446.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=248.62,462.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=153.70,438.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=98.53,462.58");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=6.94,443.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=51.43,384.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=38.89,325.58");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=48.28,523.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=33.05,265.26");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=165.51,531.15");
    }

}

