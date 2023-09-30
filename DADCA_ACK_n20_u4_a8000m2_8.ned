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

network DADCA_ACK_n20_u4_a8000m2_8
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
        @display("i=misc/sensor;p=421.45,7942.57");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1089.04,7801.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2372.33,7942.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3368.11,7995.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4658.72,7842.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5025.30,7786.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5589.73,7857.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=372.93,7576.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=765.15,7564.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1544.91,7569.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2098.20,7549.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2636.69,7598.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3408.19,7505.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3775.11,7688.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4312.75,7672.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1066.14,7372.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2945.60,7266.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4302.59,7277.23");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4643.41,7485.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5018.41,7420.92");
    }

}

