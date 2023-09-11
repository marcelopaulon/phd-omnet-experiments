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

network DADCA_ACK_n20_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=151.96,150.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=224.85,171.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=191.34,229.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=303.22,253.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=283.31,321.64");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=297.24,410.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=225.46,373.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=411.48,284.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=480.83,326.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=384.86,171.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=433.94,114.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=461.50,179.61");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=491.94,94.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=429.92,39.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=515.57,34.46");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=572.80,0.07");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=559.01,96.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=641.13,146.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=696.95,112.48");
    }

}

