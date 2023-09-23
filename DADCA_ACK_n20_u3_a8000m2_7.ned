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

network DADCA_ACK_n20_u3_a8000m2_7
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
        @display("i=misc/sensor;p=27.40,7783.15");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=956.87,7794.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1349.35,7886.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2269.66,7870.31");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3041.43,7755.53");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3346.77,7949.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4803.58,7947.09");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5541.30,7797.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=517.95,7682.02");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1765.92,7560.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3931.84,7546.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4437.65,7667.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=110.70,7274.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=818.74,7445.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1408.81,7449.57");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2153.91,7326.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2627.75,7377.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3023.84,7375.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4395.02,7258.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5087.49,7390.29");
    }

}

