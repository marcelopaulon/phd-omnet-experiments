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

network DADCA_ACK_n100_u4_a8000m2_4
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
        @display("i=misc/sensor;p=577.04,7813.32");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1720.83,7792.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2172.74,7782.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2652.56,7819.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3309.22,7941.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4996.76,7914.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1063.66,7664.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3975.62,7635.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4444.40,7594.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=39.61,7412.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1891.52,7319.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2390.12,7463.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3078.63,7475.65");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3480.99,7466.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=81.15,7046.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=522.27,7197.51");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3070.07,7111.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3744.75,7160.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4142.00,7181.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4677.58,7245.20");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=546.98,6821.54");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=960.02,6937.84");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1641.41,6957.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2142.63,7001.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2758.60,6854.46");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2399.20,6700.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3122.41,6527.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3459.16,6688.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3987.13,6685.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4357.88,6655.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4853.95,6603.97");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5246.22,6751.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5814.30,6717.40");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=58.79,6426.97");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=716.22,6494.64");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1397.91,6486.97");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1853.13,6469.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3592.20,6358.51");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4694.82,6286.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5504.64,6504.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=267.25,6072.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=634.95,6094.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1294.29,6056.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1873.22,6117.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2279.57,6092.49");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2715.05,6260.99");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3275.58,6057.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3949.84,6205.87");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5016.60,6116.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5916.33,6244.32");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3772.70,5849.14");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4246.90,5866.41");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4603.74,5795.07");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5418.56,5857.46");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=372.64,5605.34");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=992.24,5594.78");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1638.33,5751.95");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2837.24,5752.87");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5992.31,5616.67");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1268.73,5369.35");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2245.69,5405.36");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2636.17,5370.99");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3220.57,5515.85");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3567.19,5333.83");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3968.57,5418.75");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4860.65,5433.85");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5684.56,5418.12");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=99.00,5222.30");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=623.65,5239.30");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1740.74,5117.83");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2759.28,5041.65");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5233.58,5201.15");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5957.90,5122.04");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=825.45,4913.29");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1168.71,5004.91");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3355.75,4789.95");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4932.20,4793.28");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=419.82,4602.60");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1590.56,4690.97");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1971.47,4772.05");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3971.15,4752.21");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4410.19,4759.49");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=26.40,4304.05");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1014.24,4365.70");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1775.64,4385.66");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2494.63,4515.38");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3017.99,4486.37");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4200.95,4293.41");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4864.99,4435.30");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=424.70,4080.28");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2275.96,4188.60");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2797.18,4116.52");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3530.75,4286.95");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4567.12,4090.93");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5029.67,4107.38");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1209.11,4018.23");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1753.99,4028.41");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2461.86,3891.83");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3427.63,3813.90");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3938.96,3896.83");
    }

}

