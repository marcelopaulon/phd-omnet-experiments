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

network DADCA_ACK_n20_u8_a8000m2_3
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
        @display("i=misc/sensor;p=1021.39,7992.36");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1453.31,7888.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2102.50,7892.23");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2496.85,7951.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2860.01,7947.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3709.79,7764.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4125.14,7825.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4634.18,7963.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5043.22,7939.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5856.57,7893.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=698.19,7736.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1781.42,7690.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2812.31,7577.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3373.90,7636.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4629.68,7594.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5149.34,7564.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=854.20,7384.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1272.51,7435.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1805.64,7287.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2197.90,7408.98");
    }

}

