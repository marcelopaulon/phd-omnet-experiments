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
        @display("i=misc/sensor;p=981.88,7834.30");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2092.09,7950.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3220.12,7872.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5033.43,7812.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5454.38,7855.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=8.79,7676.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1577.32,7744.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3550.99,7730.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4249.56,7602.25");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5856.61,7743.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=126.62,7292.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=488.32,7377.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2004.21,7334.11");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2634.83,7502.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3299.89,7333.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4898.00,7366.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5391.79,7494.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5819.26,7342.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=812.22,7163.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1215.14,7195.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3719.19,7065.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4209.89,7236.07");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5383.84,7012.37");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=458.87,6768.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1036.32,6778.36");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1950.40,6817.28");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2389.33,6832.18");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2871.99,6881.45");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4451.53,6838.63");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4999.46,7005.98");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=25.89,6637.59");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1501.20,6756.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3529.06,6747.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3988.47,6743.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4698.72,6520.19");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5166.11,6617.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5867.37,6626.73");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=908.24,6420.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1498.85,6373.63");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2015.29,6412.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2399.58,6357.89");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2866.47,6433.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4058.26,6362.67");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5525.00,6461.56");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=94.32,6055.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=488.08,6159.27");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3387.71,6236.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4265.13,6037.70");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5022.73,6233.08");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5596.11,6087.44");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1052.40,5806.62");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1681.03,5853.93");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2141.96,5972.37");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2465.52,5781.33");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2946.86,5859.73");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3427.86,5855.06");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3845.24,6016.32");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4604.65,5881.26");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5279.84,5844.48");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=171.20,5581.70");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=536.40,5693.37");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4051.22,5534.25");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4776.96,5526.06");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5558.90,5539.53");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5843.96,5743.03");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1161.45,5460.08");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1688.17,5432.66");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2136.12,5402.24");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2884.28,5404.20");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3246.18,5440.04");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3586.34,5524.13");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4430.33,5345.28");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5148.54,5505.75");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=901.78,5201.43");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3618.75,5039.48");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5932.84,5204.52");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=302.56,5022.57");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1238.61,4933.19");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2522.97,5017.19");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3221.62,4935.91");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4677.29,4998.41");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5297.01,4969.42");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1119.85,4585.02");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1977.20,4758.11");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2910.07,4603.97");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4120.02,4781.35");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4929.52,4570.97");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5647.89,4599.36");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=577.42,4427.37");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1447.21,4456.15");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2391.46,4458.73");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3241.81,4397.45");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3832.37,4388.91");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4302.79,4420.09");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5270.22,4380.97");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=332.13,4124.18");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=999.50,4183.04");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1477.60,4072.99");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1803.14,4209.90");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2643.11,4165.48");
    }

}

