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

network DADCA_ACK_n50_u3_a8000m2_9
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
        @display("i=misc/sensor;p=593.73,7830.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2023.07,7886.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2489.36,7995.25");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3034.64,7770.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3867.43,7786.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4455.69,7927.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5152.65,7853.86");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5754.90,7982.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=82.68,7598.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1636.84,7722.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2626.67,7629.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3511.45,7747.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4218.32,7655.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=551.51,7284.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=998.07,7494.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3302.57,7466.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4658.99,7480.21");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5662.59,7383.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1100.85,7039.91");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1644.06,7053.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2066.81,7139.83");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2504.27,7088.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3213.68,7015.17");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3803.43,7106.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4430.64,7074.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=101.45,6960.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=735.43,6862.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=954.61,6571.61");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1555.12,6696.42");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2270.75,6589.46");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2711.23,6693.52");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4199.23,6562.43");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4579.88,6697.79");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5742.16,6711.12");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=215.97,6331.51");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=696.76,6269.53");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1214.48,6286.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1919.95,6291.42");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2720.51,6300.74");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3557.84,6465.23");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5317.51,6456.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1588.77,6058.40");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3777.67,6077.37");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4455.87,6065.58");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4941.60,6208.82");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5738.61,6046.63");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=71.50,5876.61");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=553.24,5853.24");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1196.60,5864.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2216.57,6002.76");
    }

}

