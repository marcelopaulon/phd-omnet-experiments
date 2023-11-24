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

network DADCA_ACK_n100_u8_a8000m2_3
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
        @display("i=misc/sensor;p=550.49,7831.82");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=961.43,7848.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1592.40,7854.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2013.85,7842.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3833.06,7873.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4452.98,7985.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5090.48,7787.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2434.39,7748.20");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2794.62,7596.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4605.80,7511.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=318.55,7471.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=805.47,7435.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1147.02,7273.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1553.61,7350.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1991.83,7377.19");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2481.63,7381.08");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3458.78,7471.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4243.05,7452.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5180.73,7364.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5682.04,7361.85");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2282.14,7045.90");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3034.18,7040.25");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3963.55,7123.62");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4516.89,7087.39");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4880.63,7124.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5454.72,7093.66");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=620.54,6870.07");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1001.54,6852.97");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1636.26,6948.20");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3495.53,6962.64");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5813.71,6976.56");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=131.46,6571.58");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1672.96,6593.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2245.31,6681.36");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2850.31,6720.25");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3289.77,6581.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3787.31,6588.37");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4466.66,6724.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4958.23,6639.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5297.02,6752.24");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5611.40,6575.43");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5974.72,6637.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=539.82,6360.95");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=918.75,6271.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1251.37,6398.82");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2276.79,6315.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2646.06,6387.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5170.10,6323.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=88.54,6075.68");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1573.17,6192.04");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3222.62,6225.25");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3605.67,6246.25");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4413.53,6121.51");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4781.17,6151.58");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5961.95,6117.10");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=398.13,5805.53");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=807.49,5878.62");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2048.29,5917.22");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2406.35,5961.18");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3423.72,5828.66");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3901.73,5900.98");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5472.35,5958.38");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1642.44,5718.59");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2930.53,5705.42");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4338.05,5578.13");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4880.26,5630.24");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5759.86,5546.90");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=132.20,5519.23");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=719.30,5465.68");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1158.95,5401.16");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2286.61,5464.72");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3201.58,5384.49");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3582.92,5451.92");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5187.67,5344.45");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1482.72,5124.24");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1962.54,5181.71");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2832.60,5234.08");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3937.88,5145.70");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4557.28,5244.48");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=425.09,4952.52");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=850.36,4920.86");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2409.70,4892.32");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3593.94,4949.55");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4253.39,4956.08");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4689.14,4907.54");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5063.08,4835.41");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5658.18,4908.07");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=152.43,4712.59");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1305.29,4718.11");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1958.14,4705.31");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2717.77,4648.04");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3238.41,4722.47");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3864.75,4548.78");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4212.26,4601.99");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=760.97,4389.26");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1797.08,4373.98");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5095.85,4470.82");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=281.89,4062.67");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1028.23,4108.73");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1420.79,4252.04");
    }

}

