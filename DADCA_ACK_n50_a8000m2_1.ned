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

network DADCA_ACK_n50_a8000m2_1
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
        @display("i=misc/sensor;p=2307.14,7940.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3425.03,7874.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3848.22,7911.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4519.57,7788.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5188.03,7867.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=76.89,7578.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=598.66,7684.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1056.28,7653.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1950.69,7638.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3004.32,7736.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3672.07,7586.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5793.34,7567.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1611.55,7360.24");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2499.92,7326.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4030.77,7288.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4414.12,7347.46");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5106.32,7409.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=261.84,7195.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=719.86,7114.26");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1294.61,7207.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2918.33,7178.94");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1651.09,6958.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2123.80,6899.56");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2545.44,6963.05");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3144.50,6862.47");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3551.73,7008.43");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4199.58,6906.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4883.34,6799.82");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5411.97,6983.65");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5880.68,7007.87");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=95.36,6742.27");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=630.99,6627.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1221.26,6730.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2493.41,6546.99");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3442.05,6661.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3859.35,6686.81");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5458.47,6544.26");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5841.00,6624.08");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=213.79,6282.86");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=997.60,6339.94");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1492.64,6491.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2115.76,6491.00");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2873.31,6352.41");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3418.99,6285.07");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4160.08,6372.37");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1316.79,6072.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3870.07,6138.02");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4317.69,6025.48");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4623.66,6220.49");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5406.98,6162.10");
    }

}

