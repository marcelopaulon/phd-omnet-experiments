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

network DADCA_ACK_n50_u16_a8000m2_5
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
        @display("i=misc/sensor;p=254.01,7945.98");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1106.33,7977.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1731.25,7883.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2104.30,7996.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2639.65,7881.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4349.23,7838.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4858.96,7960.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=587.47,7578.57");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=939.62,7612.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1316.74,7587.19");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4753.01,7598.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5635.93,7596.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=123.87,7338.85");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1745.00,7447.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2107.59,7399.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2609.92,7297.09");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2996.48,7440.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4218.34,7364.75");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5148.89,7258.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=781.47,7200.45");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1715.11,7064.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2262.55,7030.83");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3228.81,7154.85");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3582.39,7083.11");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4613.54,7226.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=341.65,6983.06");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1340.74,7007.79");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2852.27,6853.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4082.25,6909.01");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4452.28,6762.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1215.98,6592.68");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1873.35,6635.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2990.57,6522.46");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3808.78,6667.58");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4915.51,6711.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=176.44,6503.34");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=704.26,6480.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1587.91,6403.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2552.63,6375.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3395.34,6493.76");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4464.55,6302.35");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4932.31,6309.76");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5736.69,6478.50");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=112.91,6119.66");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1081.54,6249.90");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2002.14,6105.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2914.45,6152.97");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3962.05,6257.32");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5499.97,6061.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=765.24,5813.83");
    }

}

