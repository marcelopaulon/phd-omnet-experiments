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

network DADCA_ACK_n20_u4_a8000m2_0
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
        @display("i=misc/sensor;p=993.49,7753.75");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3293.17,7975.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5145.74,7969.73");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5760.21,7765.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1818.16,7585.07");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2244.94,7548.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2879.69,7699.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3334.07,7551.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4033.48,7515.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4639.70,7730.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=356.76,7279.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=825.86,7304.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1445.20,7293.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2999.10,7363.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4341.06,7323.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4783.22,7387.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5146.88,7315.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5542.85,7450.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1989.01,7170.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2667.37,7086.27");
    }

}

