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
        @display("i=misc/sensor;p=463.79,7854.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1541.66,7995.31");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2066.15,7863.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2636.15,7975.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3197.10,7970.07");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4811.67,7833.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5760.04,7960.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=15.99,7617.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=812.65,7576.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1361.40,7514.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2843.69,7630.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3277.01,7628.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3692.91,7717.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4177.29,7745.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=475.65,7444.60");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1926.62,7262.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2452.36,7332.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3683.26,7302.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4482.92,7428.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5173.08,7471.04");
    }

}

