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

network DADCA_ACK_n50_u4_a8000m2_9
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
        @display("i=misc/sensor;p=986.78,7944.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3046.95,7914.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3744.91,7897.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4632.82,7838.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5970.41,7899.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1462.27,7703.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2044.92,7560.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2591.99,7506.12");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3394.52,7511.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4265.98,7506.41");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5530.11,7634.79");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1153.47,7422.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3048.73,7361.17");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5073.73,7318.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1564.01,7236.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2046.82,7117.56");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2469.12,7144.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3736.94,7239.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5723.19,7229.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1215.53,6768.49");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3196.60,6994.92");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4196.72,6833.74");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5530.07,6934.44");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1799.85,6662.91");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2160.46,6756.14");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2759.54,6586.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3507.19,6732.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4669.38,6605.07");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=994.74,6444.14");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1445.29,6432.55");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3328.36,6296.91");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3796.01,6478.53");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5257.72,6447.94");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=797.87,6048.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1597.65,6023.85");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2355.89,6259.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2821.25,6128.59");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3750.34,6057.65");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4138.88,6261.53");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4854.72,6235.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5533.28,6195.98");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1228.64,5990.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2140.15,5916.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5239.19,5935.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=386.57,5598.62");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1011.18,5680.16");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1408.06,5573.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2666.79,5584.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3260.99,5756.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3781.64,5583.18");
    }

}

