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

network DADCA_ACK_n20_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=43.41,125.24");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=105.20,214.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=164.86,270.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=106.60,313.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=14.19,344.41");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=7.27,283.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=185.21,343.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=164.47,204.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=230.83,178.42");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=268.62,123.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=275.03,61.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=215.32,31.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=324.99,174.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=268.88,258.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=56.55,16.08");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=135.04,31.07");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=341.70,89.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=419.10,98.41");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=190.84,115.11");
    }

}

