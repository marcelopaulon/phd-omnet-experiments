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
        @display("i=misc/sensor;p=804.33,7816.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2089.35,7883.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2530.21,7869.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5288.34,7980.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=355.40,7598.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1286.49,7582.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4211.28,7590.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4901.14,7546.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5764.35,7702.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1806.02,7362.62");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4499.57,7286.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5542.01,7356.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=80.63,7057.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=808.18,7224.84");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1316.08,7152.72");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2403.99,7024.07");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4053.26,7147.21");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5931.61,7184.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=541.78,6978.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2858.79,6995.38");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3355.95,6834.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3714.78,6853.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4346.66,6799.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5157.48,6927.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5717.58,6793.97");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=249.57,6523.15");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1237.66,6728.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1764.57,6653.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2275.49,6663.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2713.84,6622.73");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3957.65,6562.84");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4871.73,6602.96");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5339.06,6524.45");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3023.10,6416.74");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3422.64,6307.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4374.05,6301.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5682.29,6396.65");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1252.24,6205.40");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2821.46,6072.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3869.34,6202.68");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4760.02,6197.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5946.47,6158.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=142.95,6016.35");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=724.97,5866.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1673.28,5971.75");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2281.18,5960.28");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3898.28,5830.02");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4439.96,5859.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5164.02,5977.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5628.31,5991.05");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1302.14,5634.47");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2821.20,5718.55");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3262.30,5609.24");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=388.85,5286.86");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=875.17,5413.32");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1771.89,5395.77");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2546.03,5495.16");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3654.69,5474.01");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4219.33,5480.31");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4612.38,5359.53");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4963.33,5429.48");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5349.94,5328.26");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5713.67,5482.19");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2140.02,5121.59");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2896.81,5161.27");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3911.60,5199.05");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=433.55,4935.27");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1445.01,4870.55");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1821.53,4849.00");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3469.51,4931.73");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4498.85,4848.35");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4909.13,4976.05");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5834.83,5024.98");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=489.11,4566.98");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1067.96,4677.90");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2379.20,4647.11");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2744.17,4776.50");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3131.68,4673.97");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3736.81,4572.50");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4197.32,4632.30");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5235.69,4688.18");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5895.34,4561.02");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=99.59,4374.88");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=848.95,4384.42");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1519.80,4326.81");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2120.88,4384.21");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2579.36,4342.91");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4765.13,4424.46");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5193.65,4337.95");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=470.06,4190.96");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4282.20,4091.09");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5576.25,4092.31");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=238.41,3914.89");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=894.62,3961.11");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1300.39,3909.17");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1921.00,3939.61");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2396.44,3843.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2786.43,3865.54");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3814.83,3918.35");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4875.38,4028.17");
    }

}

