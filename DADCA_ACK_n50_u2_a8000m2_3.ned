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

network DADCA_ACK_n50_u2_a8000m2_3
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
        @display("i=misc/sensor;p=1002.47,7805.70");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3280.47,7828.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3861.48,7836.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5169.33,7933.51");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=635.28,7746.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1548.54,7511.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2053.94,7619.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2902.58,7531.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4471.59,7751.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5692.11,7699.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1115.70,7394.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2591.91,7369.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3652.35,7435.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4230.10,7446.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5319.01,7492.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5998.18,7323.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=446.23,7222.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=798.84,7228.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2039.74,7210.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3156.37,7150.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4499.41,7036.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4981.94,7150.38");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5331.73,7050.93");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1286.84,6767.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1854.17,6858.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2786.64,6986.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3428.67,6794.09");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4131.95,6975.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4798.76,6816.44");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5691.05,6970.85");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=125.48,6656.86");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=508.39,6756.91");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=883.06,6710.43");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2514.05,6683.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3979.89,6601.31");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4341.08,6650.19");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5392.77,6574.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1235.03,6397.04");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2116.50,6373.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2521.73,6297.44");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3068.90,6474.90");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3572.68,6288.84");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4695.18,6408.28");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5738.41,6505.86");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=322.88,6143.69");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1070.13,6082.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1427.79,6091.23");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3191.01,6136.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4139.49,6104.49");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4982.62,6078.97");
    }

}

