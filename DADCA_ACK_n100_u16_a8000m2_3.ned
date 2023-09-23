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

network DADCA_ACK_n100_u16_a8000m2_3
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
        @display("i=misc/sensor;p=433.53,7776.19");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1564.58,7850.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2324.28,7950.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2685.62,7922.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3456.08,7969.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4853.49,7905.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1060.03,7608.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1917.73,7712.76");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4205.01,7723.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4670.77,7592.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5128.22,7562.52");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5507.53,7737.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=699.32,7296.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1547.87,7352.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2333.89,7492.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2986.82,7498.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3550.21,7283.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4326.19,7365.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5931.49,7425.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=963.33,7047.06");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1815.76,7118.56");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2638.09,7142.76");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5065.30,7137.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=185.42,6936.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2088.06,6792.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3018.92,6843.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4049.09,6919.21");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4890.18,6764.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5508.77,7002.84");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5951.56,6799.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1516.94,6716.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2491.45,6654.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3735.68,6672.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5326.45,6521.22");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=224.97,6494.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=846.49,6483.18");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1833.76,6423.89");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2288.88,6329.81");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3158.37,6502.91");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4271.79,6423.67");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5059.63,6283.12");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5733.38,6389.02");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=93.84,6054.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2852.91,6141.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3624.69,6161.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4418.21,6077.17");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5919.36,6059.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1816.79,5947.18");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2344.78,5838.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3242.40,6017.03");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4150.93,5840.65");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5445.43,5982.29");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=342.35,5553.55");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2098.88,5538.74");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3044.23,5609.63");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3441.56,5541.51");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4610.24,5753.69");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5010.53,5683.71");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5358.34,5575.05");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5857.19,5709.75");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1007.91,5369.83");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1586.66,5445.07");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2448.63,5487.57");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3902.33,5492.75");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4575.71,5382.98");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5021.99,5285.27");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5803.85,5359.44");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1949.42,5179.71");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2802.10,5208.14");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3652.14,5188.58");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=271.79,4994.15");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1520.31,4899.66");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2423.23,4929.40");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3184.43,4890.58");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4503.60,4853.00");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5543.60,4962.85");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=659.22,4692.31");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1149.51,4751.06");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1819.98,4658.53");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3724.17,4716.09");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5158.91,4704.34");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5845.77,4669.18");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=264.50,4533.58");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1587.79,4306.34");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1994.26,4296.14");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2322.42,4418.24");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2667.53,4529.99");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3175.27,4461.95");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4006.30,4363.73");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4468.99,4412.50");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4900.27,4402.37");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5612.72,4398.03");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=342.94,4165.73");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=939.61,4132.98");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2755.36,4062.71");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3131.98,4050.01");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3490.66,4271.32");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=688.74,3861.11");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1581.82,3905.16");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2327.23,4015.96");
    }

}

