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
        @display("i=misc/sensor;p=561.76,7926.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=987.48,7783.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1380.87,7910.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1930.93,7924.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2299.32,7976.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2742.47,7968.53");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4157.81,7962.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5003.15,7973.20");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5994.38,7817.71");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=230.53,7617.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1361.45,7555.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2853.33,7559.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3508.22,7731.86");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4179.64,7583.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5725.29,7560.24");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=877.25,7447.30");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1848.22,7495.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2323.39,7499.27");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3728.69,7269.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4721.84,7481.57");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5116.88,7297.84");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=318.59,7017.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1122.67,7038.43");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2663.56,7112.66");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3171.95,7231.37");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5838.89,7197.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1450.79,6843.94");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1856.32,6954.11");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3518.07,6972.58");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4017.30,6880.34");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4527.57,7008.97");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4937.47,6864.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5378.83,6798.86");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=940.49,6715.90");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1873.76,6529.85");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2276.56,6554.14");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2835.52,6556.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3456.06,6541.59");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=288.46,6432.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1377.21,6441.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4016.07,6286.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4499.92,6435.87");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=930.79,6195.08");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1649.50,6146.29");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2222.82,6151.03");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2748.78,6031.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3261.74,6109.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5009.25,6046.01");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5585.00,6189.20");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5985.64,6254.00");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=385.58,6004.00");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2123.02,5788.02");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3760.95,5855.32");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4271.87,5785.28");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4702.56,5819.52");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5397.24,5798.63");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=62.80,5540.48");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=616.46,5706.33");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1277.09,5538.61");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1727.00,5733.03");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3103.49,5706.35");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5770.09,5737.70");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=513.58,5336.46");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2355.49,5441.62");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2811.72,5501.98");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3653.13,5314.57");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5131.72,5508.78");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=969.30,5260.14");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1850.56,5160.59");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2325.47,5072.80");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2814.26,5150.03");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3994.50,5187.09");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4717.55,5138.59");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5722.42,5274.37");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=236.34,4981.66");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1376.30,4996.74");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3377.38,5027.71");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5392.65,5020.94");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5913.27,4834.48");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=101.59,4607.72");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=833.29,4763.92");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1684.13,4538.10");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2098.86,4580.91");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2459.25,4611.93");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2898.26,4758.20");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3660.02,4757.33");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4261.19,4643.00");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5528.16,4614.68");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=250.32,4290.39");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=600.45,4290.25");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1180.53,4342.86");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3118.92,4354.11");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3740.43,4318.09");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4931.32,4520.89");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2631.00,4228.35");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4484.67,4203.97");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5328.90,4062.52");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5702.93,4087.24");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=114.92,3893.40");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=663.51,3862.84");
    }

}

