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

network DADCA_ACK_n20_u2_a8000m2_1
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
        @display("i=misc/sensor;p=248.18,7753.82");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=775.12,7988.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1409.00,7759.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2973.88,7801.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3516.24,7931.02");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5051.41,7877.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5528.50,7866.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1866.72,7617.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2281.52,7740.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4194.69,7646.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4635.18,7602.10");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=467.84,7392.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1040.65,7394.98");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2705.26,7458.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3800.06,7278.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5228.28,7447.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5828.85,7498.11");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1370.41,7215.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1710.97,7058.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2639.51,7097.38");
    }

}

