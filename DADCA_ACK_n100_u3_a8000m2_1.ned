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

network DADCA_ACK_n100_u3_a8000m2_1
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
        @display("i=misc/sensor;p=399.26,7929.18");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=932.76,7799.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1555.33,7758.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2144.18,7833.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2579.40,7982.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3361.72,7790.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3826.48,7976.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4662.72,7827.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1891.92,7577.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3792.15,7614.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4240.63,7726.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5003.55,7709.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5661.55,7715.34");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=283.75,7407.39");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1355.41,7429.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2541.50,7355.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4248.14,7307.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4740.14,7285.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5119.14,7374.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5481.45,7386.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5881.55,7297.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=543.92,7149.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1118.54,7051.33");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1925.44,7182.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3187.65,7223.34");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3675.12,7189.89");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=98.40,6924.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1592.92,6786.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2198.46,6913.35");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2684.87,6781.16");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4216.20,6891.19");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=356.87,6537.63");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2140.77,6545.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3223.96,6644.57");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3781.60,6673.21");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4832.03,6668.80");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5966.85,6705.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=34.50,6337.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=984.00,6504.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5250.21,6282.87");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=516.03,6193.80");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1864.11,6152.24");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2741.50,6157.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3209.52,6242.11");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3841.60,6081.15");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4208.21,6230.75");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4619.46,6068.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=756.21,5901.76");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1319.84,5945.16");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2188.25,5894.80");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2546.93,5779.68");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4987.23,5871.70");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5778.31,5975.23");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1009.33,5565.63");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3020.08,5746.94");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3584.18,5753.30");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3994.71,5594.75");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4590.45,5626.55");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5182.97,5564.09");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=467.05,5404.69");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1607.17,5449.67");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2119.49,5316.11");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2503.11,5299.17");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2921.16,5379.76");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3389.35,5284.10");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5549.31,5522.06");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=207.14,5067.18");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1112.52,5098.44");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4230.07,5259.56");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4899.45,5259.38");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5586.34,5138.98");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=852.57,4835.00");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1998.51,4894.81");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3114.81,5012.11");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3806.00,5025.48");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5182.11,4988.90");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=633.10,4536.91");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2377.19,4755.60");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4034.47,4698.81");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4487.81,4673.08");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4871.80,4589.99");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5258.67,4598.29");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5945.53,4716.10");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=59.58,4424.84");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1178.33,4329.77");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1532.79,4468.06");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2260.79,4337.68");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2767.02,4385.87");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3120.40,4450.99");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3562.91,4402.79");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4480.80,4320.66");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=682.04,4069.05");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1810.60,4191.32");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2497.90,4072.53");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3748.29,4069.12");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4090.88,4253.31");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5479.12,4269.21");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5853.20,4074.35");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=226.17,3921.74");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1336.92,3977.73");
    }

}

