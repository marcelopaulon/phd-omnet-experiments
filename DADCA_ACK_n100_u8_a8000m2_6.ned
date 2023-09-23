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

network DADCA_ACK_n100_u8_a8000m2_6
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
        @display("i=misc/sensor;p=759.27,7991.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1979.45,7944.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3499.17,7967.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4262.81,7885.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5058.99,7950.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5584.10,7840.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=984.75,7636.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1621.47,7738.44");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2353.37,7663.04");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2796.58,7602.25");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3144.89,7672.57");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4764.89,7584.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=436.42,7429.65");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1348.01,7411.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3149.78,7278.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3780.64,7384.33");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4460.99,7381.05");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5215.59,7285.42");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5711.45,7409.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=908.27,7249.57");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1596.88,7061.91");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1962.32,7136.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2676.55,7213.16");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3392.65,7018.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3750.76,7018.11");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4103.48,7047.77");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4791.02,7075.08");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=221.39,6910.89");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=646.25,6947.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1225.92,6779.78");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2249.42,6931.89");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2937.84,6914.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4522.91,6799.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5620.06,6928.03");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=578.76,6532.82");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1638.92,6547.94");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3467.53,6670.76");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3985.13,6622.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4942.54,6611.90");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=883.62,6321.90");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1235.37,6345.15");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1996.63,6429.50");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2491.31,6501.31");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2872.70,6466.01");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3651.44,6316.35");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5210.40,6329.53");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5757.50,6331.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=140.82,6110.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1655.01,6024.99");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2242.72,6032.33");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3035.02,6102.97");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3980.84,6195.21");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4573.98,6159.72");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=697.58,5910.20");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5156.98,5957.39");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5982.00,5832.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=55.94,5747.21");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1406.13,5660.14");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1886.10,5756.92");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2723.64,5671.36");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3569.69,5763.62");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4053.96,5744.04");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5488.26,5748.39");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1146.15,5348.83");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1769.24,5419.69");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2339.32,5329.89");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3742.83,5298.39");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4352.28,5314.55");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4891.45,5411.01");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=640.09,5197.66");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1750.25,5053.21");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3136.77,5202.53");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5209.80,5238.67");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5728.83,5117.70");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1219.35,4830.66");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2029.22,4785.87");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4643.07,4936.55");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5054.91,4889.83");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5982.86,4806.49");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=568.57,4639.61");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2794.18,4712.40");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3235.78,4545.94");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3598.19,4782.32");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4081.75,4628.20");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5546.86,4631.98");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=71.13,4363.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=826.79,4339.15");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1721.53,4521.13");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2515.22,4399.17");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2969.61,4313.39");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4650.38,4303.54");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5027.40,4372.55");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1308.76,4177.01");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2134.81,4111.43");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3544.94,4213.26");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4026.64,4201.55");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5639.84,4263.21");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5987.72,4135.51");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=231.12,4011.51");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=636.30,3977.00");
    }

}

