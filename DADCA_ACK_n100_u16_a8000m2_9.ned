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

network DADCA_ACK_n100_u16_a8000m2_9
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
        @display("i=misc/sensor;p=856.03,7785.22");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1520.07,7940.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2237.46,7785.72");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=445.67,7733.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1821.36,7575.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2686.35,7624.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3213.16,7527.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3604.83,7695.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3965.45,7689.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4650.63,7577.91");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5973.63,7575.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=242.22,7378.41");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1167.34,7328.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4273.09,7453.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4953.98,7375.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5343.45,7443.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=788.36,7020.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1507.71,7213.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2245.77,7174.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2843.24,7195.31");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3267.19,7052.51");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3916.21,7184.49");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5158.48,7049.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5590.09,7139.68");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=369.88,7000.41");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1246.08,6929.29");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1763.44,6822.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3833.65,6828.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4460.19,6893.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2186.96,6524.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2700.59,6720.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4120.61,6591.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5064.29,6557.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5585.28,6720.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=149.43,6332.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=700.55,6373.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1213.89,6513.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1634.89,6351.15");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4737.79,6318.12");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2514.39,6202.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3001.16,6153.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3564.19,6189.26");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4244.84,6030.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5298.23,6118.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=782.73,6015.47");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1372.10,5847.66");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1855.93,5930.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=45.24,5749.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2158.33,5638.37");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2533.02,5722.86");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3301.56,5638.27");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3816.83,5557.03");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4600.75,5723.49");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=907.78,5480.72");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2961.91,5416.59");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5186.50,5433.49");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5538.39,5415.94");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=42.04,5182.30");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=586.20,5244.06");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2000.95,5128.71");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2562.63,5121.55");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3255.40,5205.70");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4659.77,5162.10");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5772.73,5034.54");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=56.50,4808.83");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1325.45,5022.55");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2938.55,4810.33");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4006.38,5001.12");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5190.98,5023.70");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=562.03,4541.92");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=959.83,4763.48");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1874.62,4697.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2567.81,4758.39");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4074.07,4619.20");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4684.22,4687.81");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5062.71,4670.69");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=227.32,4323.64");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=955.12,4377.82");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1422.54,4429.39");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2173.69,4397.32");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2808.69,4483.67");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3401.53,4442.26");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4507.69,4309.08");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5497.98,4453.32");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5849.27,4359.13");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=613.38,4119.80");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2535.82,4256.20");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3107.10,4158.56");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3796.69,4145.81");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1641.26,3845.84");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2145.47,3857.48");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4188.02,3986.15");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4704.03,3938.34");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5330.87,3963.61");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5990.40,4022.68");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=964.53,3778.04");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1348.67,3592.78");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2624.82,3778.76");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3359.86,3759.82");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3750.37,3604.01");
    }

}

