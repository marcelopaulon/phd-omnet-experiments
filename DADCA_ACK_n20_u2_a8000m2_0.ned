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

network DADCA_ACK_n20_u2_a8000m2_0
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
        @display("i=misc/sensor;p=392.18,7870.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=822.12,7842.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1362.06,7759.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2735.79,7985.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3836.05,7985.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4351.06,7897.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5490.87,7822.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2193.78,7621.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2918.86,7671.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3348.68,7681.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3739.00,7513.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4753.92,7564.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5155.14,7538.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5971.06,7740.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=227.09,7261.45");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=758.74,7462.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1120.29,7289.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1534.30,7422.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2030.15,7290.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2387.55,7258.46");
    }

}

