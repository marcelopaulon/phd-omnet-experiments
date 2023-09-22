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

network DADCA_ACK_n100_a8000m2_6
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
        @display("i=misc/sensor;p=262.77,7743.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2936.83,7917.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3894.55,7944.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4368.38,7899.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4734.83,7946.98");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=876.89,7683.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1463.19,7531.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2185.48,7638.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2648.23,7497.93");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3297.75,7648.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4498.51,7544.97");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5270.75,7537.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=902.75,7316.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2331.84,7241.28");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3011.10,7388.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3754.59,7281.57");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5915.32,7367.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1684.39,7125.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3431.55,7012.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5018.44,7157.04");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5982.06,7007.35");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1728.05,6709.71");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2188.69,6727.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3048.28,6753.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4717.87,6865.86");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5648.04,6797.47");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1126.59,6703.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3939.92,6637.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4746.59,6478.35");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5143.91,6577.50");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1434.09,6430.91");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1822.48,6198.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2276.82,6256.44");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2813.58,6418.55");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4342.56,6445.23");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5566.36,6285.94");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=912.72,6185.00");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3106.56,6103.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3807.53,6108.79");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4743.50,5944.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5166.11,6068.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=561.26,5758.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1294.59,5836.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2514.20,5733.46");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4302.64,5797.89");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5759.92,5834.99");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=303.32,5466.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1828.71,5619.48");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3645.05,5583.90");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1308.28,5172.08");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3235.74,5290.88");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4025.70,5319.98");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4685.59,5266.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5154.84,5388.72");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5695.89,5401.47");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=175.12,5001.71");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=740.92,5151.61");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1681.91,5096.00");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2303.78,5006.22");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2680.07,5128.86");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4874.96,4939.18");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5342.12,5063.76");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5861.37,5059.97");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1930.21,4772.46");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3070.92,4840.07");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3638.75,4779.09");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4345.52,4782.93");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=522.87,4630.11");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1011.97,4569.47");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1536.53,4415.48");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2104.78,4407.31");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2737.81,4515.91");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3219.36,4377.74");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4601.63,4420.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5045.20,4552.96");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5506.58,4558.36");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=48.39,4350.53");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4118.61,4232.07");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5350.73,4173.25");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5888.57,4371.44");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=637.48,3918.91");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1356.52,3892.34");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1848.53,4059.06");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2516.58,3979.25");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3093.07,3920.39");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3797.49,4010.91");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4995.34,3979.87");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=246.55,3721.32");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1007.15,3748.34");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1731.53,3618.42");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2166.05,3780.08");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4428.65,3817.65");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5753.32,3642.63");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=669.77,3540.25");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2385.90,3431.26");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3299.39,3599.34");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3748.20,3523.51");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5077.79,3589.85");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1377.53,3148.88");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1983.91,3258.51");
    }

}

