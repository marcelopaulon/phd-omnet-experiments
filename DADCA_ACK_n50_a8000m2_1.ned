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

network DADCA_ACK_n50_a8000m2_1
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
        @display("i=misc/sensor;p=752.77,7843.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1366.81,7851.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1732.58,7872.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2400.84,7872.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2923.57,7814.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3611.58,7956.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4557.18,7846.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5260.94,7822.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=178.53,7698.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4171.05,7717.91");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5770.47,7544.81");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=676.76,7369.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1147.19,7328.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1632.57,7396.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2027.08,7330.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3444.80,7443.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4775.05,7271.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5265.82,7273.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=155.33,7198.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1573.03,6987.60");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2548.26,7196.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3174.34,7077.54");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3754.46,7136.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4266.37,7059.11");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5561.46,7035.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=379.94,6734.70");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=781.81,6945.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2145.98,6800.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2863.19,6772.01");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4642.45,6873.17");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=997.39,6522.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2570.38,6484.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3871.95,6594.14");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5072.89,6587.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=648.47,6293.26");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1600.36,6405.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2140.74,6323.10");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3155.57,6357.64");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4534.75,6289.21");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5845.02,6433.12");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=205.70,6033.50");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1614.52,6018.51");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2380.53,6030.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3542.63,6186.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4160.66,6140.35");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5284.96,6103.36");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5899.67,5931.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=806.31,5879.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1202.88,5921.61");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2027.12,5868.13");
    }

}

