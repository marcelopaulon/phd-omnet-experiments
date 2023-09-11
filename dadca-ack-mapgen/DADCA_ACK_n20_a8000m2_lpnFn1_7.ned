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

network DADCA_ACK_n20_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=34.85,38.39");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=101.79,7.96");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=59.19,212.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=9.09,282.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=93.26,317.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=158.19,331.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=125.17,411.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=41.40,382.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=112.12,248.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=193.18,191.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=1.47,184.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=254.36,417.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=318.21,359.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=224.92,351.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=373.02,319.05");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=390.06,254.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=467.23,270.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=405.24,403.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=469.78,422.72");
    }

}

