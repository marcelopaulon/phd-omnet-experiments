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

network DADCA_ACK_n20_u16_a8000m2_4
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
        @display("i=misc/sensor;p=174.51,7971.12");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=642.54,7879.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1297.44,7854.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1659.78,7753.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2145.57,7865.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2703.45,7757.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3194.66,7835.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3801.02,7998.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4988.98,7784.02");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=656.90,7514.67");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3630.34,7513.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3990.53,7600.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5617.60,7677.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=59.01,7478.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=968.15,7327.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1572.49,7361.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1970.84,7442.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2946.75,7332.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4355.72,7444.71");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5024.67,7311.49");
    }

}

