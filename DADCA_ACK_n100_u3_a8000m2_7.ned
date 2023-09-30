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

network DADCA_ACK_n100_u3_a8000m2_7
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
        @display("i=misc/sensor;p=2545.08,7954.90");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2944.38,7990.03");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3450.96,7862.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4009.82,7905.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4689.08,7984.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5476.72,7819.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1509.14,7601.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2862.37,7522.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3734.22,7577.93");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4214.46,7522.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=985.32,7488.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1818.47,7382.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2312.68,7475.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3367.99,7500.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4995.68,7460.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5504.70,7469.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=301.96,7228.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1054.49,7116.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3160.94,7044.78");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3683.59,7157.21");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4066.73,7018.82");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4690.31,7063.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5257.68,7093.80");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5941.27,7228.05");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1646.17,6957.04");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2825.65,6939.52");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3706.92,6767.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5639.77,6929.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=407.60,6710.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1062.93,6748.70");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2183.26,6746.14");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3065.12,6623.05");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4247.30,6519.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4789.50,6691.53");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5444.90,6548.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5948.60,6647.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=12.06,6268.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=862.06,6441.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1552.94,6488.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2407.21,6400.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3837.52,6439.13");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1359.14,6186.41");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2015.28,6073.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2968.26,6106.39");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3537.72,6166.79");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4502.85,6218.22");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5175.27,6075.99");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5897.05,6095.59");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=826.16,5927.44");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2337.24,5833.68");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3233.89,5851.81");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3886.88,6019.76");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4984.04,5781.80");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5644.95,5821.62");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=334.96,5768.05");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1292.15,5624.71");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1991.73,5542.69");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2627.58,5553.13");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4015.98,5552.33");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=177.84,5327.18");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=737.41,5524.55");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1570.33,5410.93");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3068.82,5374.80");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3576.61,5348.61");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4652.58,5384.25");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5106.10,5403.75");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=579.37,5127.19");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1020.31,5047.08");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2163.53,5131.77");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4925.59,5090.97");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1581.86,5027.03");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2454.39,4847.07");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3141.18,4839.52");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3805.24,4865.35");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=124.34,4657.65");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=506.77,4543.74");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1178.09,4607.41");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4089.07,4586.81");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4442.91,4671.80");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5062.32,4756.68");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5388.34,4568.98");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1539.08,4401.28");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1896.13,4532.87");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2794.93,4504.23");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3106.15,4333.05");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4827.77,4474.35");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5144.51,4315.25");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5882.05,4332.27");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=524.12,4110.52");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1047.58,4124.29");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2280.91,4270.34");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3814.89,4176.51");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4488.96,4256.31");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1437.69,4020.08");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1963.97,3794.59");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3261.02,3830.49");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3622.47,3817.26");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4162.15,3813.52");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5506.30,3982.68");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=973.82,3724.94");
    }

}

