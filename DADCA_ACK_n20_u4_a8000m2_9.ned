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

network DADCA_ACK_n20_u4_a8000m2_9
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
        @display("i=misc/sensor;p=1074.92,7908.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1669.77,7989.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2554.14,7863.47");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4274.63,7753.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4995.78,7921.54");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5954.04,7937.96");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1581.79,7529.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1958.26,7601.76");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3344.11,7679.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3748.29,7549.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5385.47,7547.88");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=701.56,7441.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1154.64,7266.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2807.82,7491.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4484.55,7473.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5903.24,7322.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=47.88,7189.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1769.15,7226.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2518.06,7125.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3327.97,7076.64");
    }

}

