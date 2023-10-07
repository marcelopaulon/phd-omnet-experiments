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

network DADCA_ACK_n20_u2_a8000m2_6
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
        @display("i=misc/sensor;p=718.03,7823.02");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1259.37,7796.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2562.78,7872.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3402.36,7972.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4857.01,7828.04");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5503.16,7885.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5893.05,7856.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=188.20,7605.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1872.10,7726.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2256.90,7657.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2894.58,7708.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3341.51,7606.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3800.68,7585.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5249.49,7635.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=904.96,7457.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1346.76,7328.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2007.53,7359.02");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4471.70,7494.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4897.25,7281.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5796.17,7479.61");
    }

}

