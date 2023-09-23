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

network DADCA_ACK_n20_u2_a8000m2_9
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
        @display("i=misc/sensor;p=1078.19,7753.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2548.89,7779.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3101.82,7874.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4372.03,7779.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4787.10,7968.42");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1511.67,7715.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1935.94,7711.77");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3411.96,7580.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3921.60,7658.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5501.22,7541.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=586.17,7347.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1779.27,7335.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2167.48,7309.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2516.58,7283.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2935.33,7315.29");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3809.84,7302.37");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4674.77,7504.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5155.68,7426.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1396.27,7230.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4454.72,7157.01");
    }

}

