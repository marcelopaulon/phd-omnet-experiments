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

network DADCA_ACK_n20_u16_a8000m2_5
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
        @display("i=misc/sensor;p=373.28,7787.06");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=884.81,7797.39");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2194.13,7831.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3068.01,7818.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3622.63,7822.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4372.31,7841.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1436.38,7666.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3967.81,7750.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4830.81,7540.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=93.93,7493.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=592.50,7460.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1121.60,7355.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1768.03,7297.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2253.59,7483.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2627.70,7379.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3100.95,7334.57");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3474.68,7460.46");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3884.15,7349.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4435.23,7278.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5348.50,7500.40");
    }

}

