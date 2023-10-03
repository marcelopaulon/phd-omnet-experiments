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

network DADCA_ACK_n100_u8_a8000m2_5
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
        @display("i=misc/sensor;p=52.10,7879.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1078.07,7831.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3061.32,7784.28");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3475.37,7824.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4357.99,7936.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4827.18,7802.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5874.73,7910.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=508.54,7574.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2042.97,7512.40");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3925.74,7612.41");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5480.87,7643.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1036.75,7334.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1644.24,7449.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2330.97,7310.92");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2667.49,7478.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3291.58,7469.33");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4158.79,7332.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4592.64,7373.54");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4952.30,7404.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5915.61,7332.11");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=309.39,7244.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=823.99,7021.43");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3507.46,7185.63");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1440.90,6863.82");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2158.68,6781.35");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2680.82,6912.96");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3984.81,6803.30");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5350.71,6966.51");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5841.15,6974.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=250.41,6571.65");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=572.20,6732.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=916.72,6585.53");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3189.63,6738.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4452.61,6638.74");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4888.66,6734.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=542.83,6306.53");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1542.80,6361.28");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2693.00,6360.81");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4071.46,6455.60");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5363.05,6457.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5978.79,6425.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=252.92,6083.79");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=919.38,6193.24");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1266.65,6116.06");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2127.39,6201.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3040.06,6206.61");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3620.58,6124.36");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4557.20,6163.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4939.39,6184.39");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5389.60,6050.53");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1839.64,5878.26");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2653.15,5799.09");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4185.35,5823.88");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=76.94,5589.44");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=457.01,5728.59");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=978.54,5583.11");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2224.91,5666.44");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3262.63,5753.98");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4948.79,5640.84");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5341.23,5666.81");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5809.45,5768.34");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2608.94,5416.78");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3054.81,5431.70");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3439.79,5423.21");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3942.97,5392.16");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4431.94,5415.94");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5904.87,5337.62");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=226.57,5202.81");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=690.64,5277.57");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1031.53,5144.18");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1488.03,5138.49");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2007.60,5186.51");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4755.82,5208.47");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5347.72,5261.28");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=228.60,4793.59");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3003.41,4983.48");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3665.23,4812.92");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4022.22,4968.02");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4546.93,4869.05");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5234.42,4905.83");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=733.12,4564.89");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2003.37,4723.40");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2442.81,4576.42");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3223.39,4671.59");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4285.49,4584.24");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4878.51,4689.13");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5601.55,4647.31");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1255.81,4523.49");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=195.64,4205.73");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=855.42,4195.16");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1848.00,4251.61");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2282.28,4237.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2779.12,4125.98");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3625.79,4207.17");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4161.65,4091.58");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4703.13,4239.80");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5183.57,4106.31");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5872.09,4132.99");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1319.70,3907.73");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2016.28,3934.75");
    }

}

