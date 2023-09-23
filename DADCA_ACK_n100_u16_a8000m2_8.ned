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

network DADCA_ACK_n100_u16_a8000m2_8
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
        @display("i=misc/sensor;p=367.45,7756.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2391.14,7834.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3052.19,7765.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3403.70,7888.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3953.41,7781.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4357.46,7872.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4938.71,7996.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=848.50,7652.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1502.77,7653.24");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1890.71,7600.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5631.28,7577.67");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=163.06,7354.83");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1347.31,7338.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3604.72,7389.04");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4008.29,7376.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4390.42,7468.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5047.37,7306.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5995.17,7303.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=507.13,7048.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=861.87,7155.70");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1757.23,7143.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2140.99,7207.07");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2716.80,7238.40");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3276.90,7208.05");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4578.37,7065.32");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=158.04,6787.45");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1192.93,6957.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3352.59,6806.31");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3744.83,6781.17");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4139.12,6990.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5413.96,6954.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5993.06,6915.96");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=788.26,6672.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1522.72,6714.91");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2393.68,6547.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2800.16,6588.21");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4332.69,6676.11");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4926.21,6716.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5.81,6447.44");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1074.58,6427.87");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1892.21,6458.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3287.80,6444.13");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3692.06,6431.43");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5133.62,6331.00");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5787.91,6503.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=298.33,6128.41");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=812.42,6031.43");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1398.45,6038.57");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2181.05,6063.89");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2841.43,6186.10");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3952.28,6127.69");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4713.19,6213.84");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5376.44,6074.19");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5916.15,6111.28");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1785.76,6011.73");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2431.75,5806.88");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3400.47,5839.15");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4497.87,5799.01");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=131.35,5712.38");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=614.73,5576.85");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1342.62,5665.76");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2862.78,5535.76");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3895.39,5650.67");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4913.54,5589.41");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5255.68,5707.43");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1029.15,5326.74");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2089.34,5410.99");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3448.33,5379.86");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3816.80,5293.41");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4403.64,5399.16");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5688.65,5513.03");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=122.74,5254.92");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1446.60,5038.53");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2669.48,5177.87");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5442.95,5229.36");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5798.40,5159.99");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=396.02,4873.53");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1054.35,4829.15");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2059.19,4945.33");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3226.91,4887.22");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3640.98,4950.02");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4797.85,5029.37");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5190.71,4922.64");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2547.95,4761.88");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2902.57,4674.75");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5811.76,4765.29");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=100.45,4512.32");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1445.00,4482.62");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1819.54,4494.08");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2219.27,4462.76");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3250.35,4417.08");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3856.97,4379.32");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4242.09,4333.95");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4898.26,4400.28");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5476.61,4338.36");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=675.99,4285.15");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2179.18,4067.35");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2920.24,4143.73");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3450.47,4113.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5754.46,4125.15");
    }

}

