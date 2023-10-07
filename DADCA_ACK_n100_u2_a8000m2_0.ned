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

network DADCA_ACK_n100_u2_a8000m2_0
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
        @display("i=misc/sensor;p=1115.32,7857.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1786.19,7784.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3483.09,7768.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3899.11,7757.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4486.97,7825.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5390.40,7866.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5862.45,7844.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=333.47,7691.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2284.59,7637.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3058.22,7696.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=689.01,7500.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1498.08,7398.73");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2111.71,7306.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2695.84,7468.11");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3688.87,7473.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4072.65,7406.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5488.62,7316.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=74.31,7246.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=455.25,7025.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1118.97,7186.34");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2574.26,7099.73");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3225.15,7247.58");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3600.92,7080.43");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4289.71,7095.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4884.27,7181.46");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5951.14,7196.19");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1357.40,6802.95");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2114.82,6873.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2858.68,6787.75");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5216.79,6847.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=169.19,6656.71");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=663.88,6720.35");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1706.18,6647.27");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3156.80,6581.60");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3809.29,6634.77");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4442.31,6598.70");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5813.21,6747.57");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1144.90,6501.67");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1449.44,6320.53");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2434.91,6505.65");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2855.45,6387.93");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3508.47,6355.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5089.31,6506.89");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5594.87,6378.96");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=40.44,6057.07");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1931.97,6102.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2564.26,6081.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3813.75,6147.19");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5134.55,6082.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=598.42,5999.84");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3336.41,5943.60");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4423.66,6001.68");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=242.39,5650.51");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=605.00,5598.58");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1117.34,5753.53");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1495.16,5684.98");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2065.30,5607.94");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3738.13,5711.24");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5144.68,5572.62");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5508.87,5657.67");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1262.00,5297.07");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1733.54,5406.68");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2417.10,5476.46");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2778.61,5524.07");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3768.40,5338.98");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4356.29,5409.86");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4805.00,5452.12");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=599.99,5101.89");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2288.66,5143.20");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3361.74,5249.73");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5604.59,5138.08");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5989.65,5087.78");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=98.40,5006.11");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1246.01,4844.73");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1923.16,4989.31");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=503.18,4598.57");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=881.78,4679.50");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1674.97,4585.04");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2476.32,4780.89");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3425.03,4597.17");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3988.55,4763.37");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4747.02,4776.12");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5343.40,4759.93");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=53.14,4457.17");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2030.77,4329.88");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2775.51,4419.08");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4643.38,4411.71");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5006.15,4407.31");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5393.90,4349.45");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5809.71,4496.43");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=549.59,4108.80");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1036.96,4274.58");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1413.22,4059.62");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2600.66,4048.51");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3141.58,4205.62");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3994.28,4205.63");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=215.58,3961.66");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1788.49,3940.78");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2210.20,4000.87");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3767.78,3923.91");
    }

}

