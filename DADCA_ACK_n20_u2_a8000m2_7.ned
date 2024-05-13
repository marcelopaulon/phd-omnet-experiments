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

network DADCA_ACK_n20_u2_a8000m2_7
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
        @display("i=misc/sensor;p=9.45,7870.48");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1476.08,7902.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1868.14,7913.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3402.59,7897.77");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4067.30,7864.15");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4547.72,7960.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5745.23,7880.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=376.47,7663.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=778.25,7702.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1219.10,7588.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2825.82,7574.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5263.35,7607.18");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=112.49,7330.38");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=735.80,7312.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1790.32,7442.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2166.62,7430.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3424.51,7448.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3944.62,7272.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4396.81,7355.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5772.85,7347.30");
    }

}

