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

network DADCA_ACK_n100_u8_a8000m2_0
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
        @display("i=misc/sensor;p=266.81,7853.11");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=630.70,7937.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1222.77,7999.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2093.44,7984.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2463.95,7758.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2980.54,7923.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5017.24,7791.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5536.82,7900.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1084.05,7536.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1792.96,7655.43");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2163.05,7560.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2974.22,7507.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3555.77,7660.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4089.90,7583.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=410.67,7435.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2629.04,7339.86");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3306.37,7358.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4854.85,7280.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5719.21,7284.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1025.59,7144.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1493.11,7136.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1974.66,7145.08");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3811.37,7065.68");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5198.39,7120.64");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=153.93,6956.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4405.44,6894.10");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5950.88,6814.73");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=697.37,6707.64");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1561.45,6684.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2628.89,6677.61");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3052.34,6556.17");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3645.06,6617.35");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4088.38,6617.73");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4788.09,6724.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5250.51,6633.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5604.83,6705.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=337.75,6472.73");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1175.30,6514.72");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1954.27,6481.97");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4777.48,6286.67");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5994.82,6275.61");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=832.24,6191.03");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1493.09,6266.61");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2236.98,6062.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2688.16,6226.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3846.69,6160.41");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4409.67,6154.79");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5331.38,6189.24");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=511.48,5931.41");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1358.30,5806.47");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1807.84,5938.59");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2969.40,5968.38");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4980.59,5793.80");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=732.20,5618.76");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3487.74,5770.12");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4295.99,5737.24");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5463.69,5725.01");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=319.87,5342.47");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1073.30,5442.61");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1764.43,5409.25");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2354.73,5491.26");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2924.65,5370.72");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3980.88,5434.46");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4523.03,5360.89");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=697.55,5051.15");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1350.55,5136.18");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1889.37,5039.84");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3619.39,5116.00");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5046.40,5168.71");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5557.16,5103.79");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=283.44,4942.40");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2488.03,4807.84");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3098.78,5011.11");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4232.04,4835.43");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5903.75,4927.66");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=722.87,4612.35");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1384.21,4705.42");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1797.56,4686.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4961.96,4659.98");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5602.13,4654.74");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1038.62,4428.71");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1643.20,4313.70");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2615.79,4411.01");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3021.74,4397.67");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3599.20,4484.34");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3965.46,4458.00");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5199.37,4374.29");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=220.30,4234.59");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=596.64,4093.93");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2006.32,4108.23");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3679.71,4085.87");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4211.44,4177.46");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4869.07,4190.43");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5511.26,4152.63");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5883.78,4234.58");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=973.74,3970.62");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1360.40,3969.61");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2677.69,3949.59");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4928.37,3815.90");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=168.21,3786.08");
    }

}

