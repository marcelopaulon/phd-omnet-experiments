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

network DADCA_ACK_n100_u16_a8000m2_7
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
        @display("i=misc/sensor;p=242.25,7790.88");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=777.60,7925.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1518.04,7802.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1899.66,7892.20");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2669.81,7764.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3157.37,7832.03");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3608.66,7885.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4026.65,7899.51");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4544.07,7841.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5688.56,7775.42");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3919.96,7546.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5206.49,7740.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=577.30,7294.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2207.84,7433.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3379.99,7497.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4577.66,7420.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4940.91,7264.90");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5623.00,7361.37");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1279.37,7121.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1649.91,7237.34");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2944.70,7254.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3355.02,7023.09");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4320.91,7031.03");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=644.68,6819.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1605.71,6785.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2495.40,6936.45");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3057.10,6777.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5178.65,6852.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5676.40,6893.37");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=104.54,6554.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1971.91,6756.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2583.13,6515.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3642.48,6754.40");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4548.41,6745.61");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5978.34,6518.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1070.26,6427.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1986.43,6313.00");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3929.58,6500.58");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4575.06,6309.22");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5517.61,6287.85");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=269.22,6147.14");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=729.92,6264.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2616.00,6124.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3258.63,6235.48");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5115.81,6228.96");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=650.38,5871.10");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1193.42,5852.37");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2790.87,5788.13");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3306.11,5805.74");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3667.72,5982.05");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4146.18,5881.88");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4985.98,5788.19");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5399.03,5854.71");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=368.20,5627.81");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=901.00,5551.05");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1716.61,5740.88");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2186.95,5756.44");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3857.17,5536.72");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4529.51,5547.98");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=55.41,5451.36");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1662.68,5313.13");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2503.83,5470.07");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3075.66,5379.53");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5207.00,5498.55");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5630.48,5428.79");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5988.73,5414.18");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=647.68,5239.14");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1175.99,5160.47");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2213.38,5213.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2775.25,5092.37");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3563.16,5067.81");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4099.42,5197.24");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4548.64,5181.75");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4979.76,5196.61");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=28.91,4994.45");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1750.06,4844.98");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2346.19,4805.54");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5761.74,4999.43");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=613.34,4571.71");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1202.49,4593.67");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3133.32,4563.35");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4485.82,4558.88");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5067.09,4704.56");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=35.00,4402.57");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1694.34,4425.97");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2153.44,4383.04");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2584.18,4440.56");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5660.48,4523.95");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=354.64,4111.18");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=735.45,4184.89");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1115.66,4155.91");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3639.96,4261.66");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5422.09,4185.48");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=873.53,3840.73");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1540.27,3994.89");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2119.64,3819.35");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3008.66,4004.52");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3303.15,3813.67");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3711.56,3825.55");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4107.69,3861.37");
    }

}

