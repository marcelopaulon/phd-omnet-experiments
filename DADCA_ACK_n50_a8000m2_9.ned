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

network DADCA_ACK_n50_a8000m2_9
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
        @display("i=misc/sensor;p=298.79,7754.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=886.08,7945.64");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2226.62,7836.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3932.52,7754.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4557.96,7926.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1843.88,7568.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2545.45,7633.68");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3354.96,7667.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5798.82,7723.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=497.23,7343.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=983.55,7413.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1499.24,7394.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2745.13,7311.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3666.13,7459.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3971.64,7244.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4586.44,7470.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4965.87,7466.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5431.51,7434.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1534.22,6966.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1952.23,7159.42");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2286.73,7004.86");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3294.40,7062.61");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4638.74,6970.78");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5121.65,7113.34");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5881.34,7172.16");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4193.38,6931.67");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5660.78,6817.38");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2591.02,6633.37");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3791.85,6673.90");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4671.46,6582.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5110.42,6558.47");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1415.69,6377.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1890.34,6399.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2458.78,6229.09");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2856.19,6269.67");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3326.02,6223.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3775.31,6205.53");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4123.63,6350.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4481.86,6265.80");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4960.50,5987.20");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5377.16,6144.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=215.07,5917.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2072.13,5912.27");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2670.77,5910.34");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3288.47,5803.10");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3971.91,5725.36");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4313.76,5883.70");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=353.79,5517.82");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1727.80,5464.58");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2805.93,5451.64");
    }

}

