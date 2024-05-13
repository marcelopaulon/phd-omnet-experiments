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

network DADCA_ACK_n100_u3_a8000m2_9
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
        @display("i=misc/sensor;p=791.55,7930.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1531.76,7971.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2188.74,7944.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3117.56,7925.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3751.13,7961.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4043.33,7768.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5942.90,7843.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=318.97,7604.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1127.50,7644.40");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1734.19,7557.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2101.15,7527.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2494.05,7715.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3523.31,7529.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4521.13,7543.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5365.66,7715.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5699.19,7556.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2463.48,7284.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2925.84,7488.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4854.61,7433.66");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=601.50,7043.78");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1377.97,7106.01");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3486.20,7100.84");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3838.18,7171.90");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4494.61,7174.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5021.57,7102.47");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5562.93,7210.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=231.44,6839.64");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2030.59,6930.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2653.03,6954.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3045.67,6907.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1694.42,6707.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2510.41,6591.48");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3631.96,6578.50");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3970.96,6729.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4638.16,6681.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5628.87,6554.89");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=430.24,6394.78");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=852.80,6480.51");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1354.40,6458.91");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2107.44,6507.93");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3050.81,6315.36");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3926.08,6351.42");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4295.55,6274.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5242.33,6489.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=807.38,6101.07");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1371.94,6096.37");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1866.87,6243.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2528.23,6053.32");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3479.01,6245.41");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4847.35,6213.20");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5812.67,6052.40");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=132.69,5885.86");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2122.78,5880.37");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3590.52,5891.73");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5204.07,5798.08");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=973.23,5730.99");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1403.20,5619.54");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1750.18,5546.59");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2979.85,5659.30");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3989.88,5759.78");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4363.26,5678.10");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4848.79,5528.09");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5768.65,5695.29");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=698.49,5490.58");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2523.78,5339.02");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4157.89,5299.19");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5321.64,5420.51");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=335.58,5081.78");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1241.01,5174.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1966.60,5244.74");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3294.95,5120.22");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3804.08,5047.36");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5840.71,5274.61");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1683.58,4812.51");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2294.19,4998.43");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4574.84,4892.91");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5082.34,4995.14");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=866.66,4662.69");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2466.53,4673.86");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2880.20,4689.98");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4229.50,4667.73");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4923.72,4612.09");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5522.03,4690.11");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=291.32,4382.05");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1181.31,4339.57");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1831.21,4477.49");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2864.29,4331.09");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3499.83,4499.13");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5899.97,4367.51");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2373.21,4217.91");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3171.32,4090.67");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4064.27,4100.34");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4656.42,4094.80");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5301.51,4150.86");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=92.55,3985.37");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=854.05,4009.94");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1478.27,4020.90");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1883.72,3847.16");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3714.10,4017.65");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5735.67,3939.67");
    }

}

