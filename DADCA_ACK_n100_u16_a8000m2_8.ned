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
        @display("i=misc/sensor;p=1236.37,7997.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1829.15,7873.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3797.00,7761.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4096.71,7945.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5081.24,7780.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5830.59,7940.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=125.97,7664.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=756.36,7544.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1418.26,7618.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2227.38,7515.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2647.82,7567.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3031.42,7747.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4750.37,7641.16");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=386.58,7385.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1190.71,7340.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1769.78,7331.60");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3429.70,7493.67");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3990.31,7450.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5322.05,7374.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5864.54,7257.64");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=685.26,7056.68");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2096.90,7017.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3714.11,7140.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4420.49,7237.39");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4789.73,7199.01");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2754.01,6968.69");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3363.99,6808.97");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5157.12,6890.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5518.52,6948.69");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=220.19,6745.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=646.69,6642.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1166.71,6644.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1819.21,6751.88");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3602.52,6547.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4174.01,6699.09");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4586.23,6539.53");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5394.49,6564.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5984.63,6610.16");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1788.17,6303.30");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2476.46,6446.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2811.68,6305.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3217.54,6322.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4228.58,6345.75");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5697.76,6384.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=348.51,6191.56");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=766.89,6177.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1345.88,6052.95");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3759.03,6178.12");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4561.89,6188.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5176.22,6197.16");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=723.36,5829.67");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1740.21,5871.17");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2694.29,5775.85");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4025.67,5945.65");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5438.39,5924.03");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1246.20,5528.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3229.11,5749.26");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3616.15,5679.64");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4488.17,5734.51");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4866.85,5722.41");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=391.92,5461.39");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1793.75,5504.54");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2186.96,5459.44");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2719.64,5373.69");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3214.64,5346.24");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4246.64,5355.15");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4929.14,5356.01");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5491.58,5486.37");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5893.00,5493.93");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=711.14,5258.39");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1053.61,5170.61");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5563.37,5141.59");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2226.93,4974.78");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2598.18,4988.26");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3012.69,4946.65");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3489.33,4887.30");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3866.47,4899.18");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4449.36,4865.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5697.13,4814.22");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=415.68,4757.96");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1017.53,4730.68");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1490.65,4776.08");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2056.03,4660.37");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2594.53,4593.36");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3192.31,4619.84");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4196.23,4617.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4982.68,4773.83");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3656.07,4342.55");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5570.31,4463.60");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5975.18,4359.00");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=714.17,4249.78");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1109.27,4224.29");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1941.27,4256.24");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2868.83,4211.15");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3305.83,4229.84");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4316.43,4287.66");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5248.50,4198.66");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=220.43,3799.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=902.30,3801.90");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1280.77,3902.33");
    }

}

