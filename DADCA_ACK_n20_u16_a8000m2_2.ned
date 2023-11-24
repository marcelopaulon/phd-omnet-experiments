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

network DADCA_ACK_n20_u16_a8000m2_2
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
        @display("i=misc/sensor;p=53.04,7869.35");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1492.11,7984.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2152.00,7874.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2776.85,7944.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3147.77,7968.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3704.15,7759.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4341.21,7772.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4855.69,7901.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5225.26,7949.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5831.73,7952.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1378.20,7537.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1818.21,7633.68");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2467.17,7680.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5262.77,7561.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=64.34,7345.43");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=547.16,7503.99");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=970.53,7371.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3129.57,7431.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3648.10,7380.57");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4145.77,7437.84");
    }

}

