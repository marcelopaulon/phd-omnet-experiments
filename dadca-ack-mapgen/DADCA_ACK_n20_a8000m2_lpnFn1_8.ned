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

network DADCA_ACK_n20_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=169.94,39.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=221.54,106.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=124.20,166.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=84.63,229.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=82.56,292.62");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=151.68,249.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=20.81,87.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=57.04,22.02");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=226.48,198.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=14.71,192.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=76.16,421.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=135.86,356.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=183.91,302.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=34.80,531.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=102.88,556.73");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=186.61,548.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=221.70,476.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=165.71,416.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=113.25,486.98");
    }

}

